import cv2
import numpy as np
import json
import os
import time
import math
import chess
import chess.engine
from arm import ChessRobotArm

# --- CONFIG ---
ROBOT_CALIB = "calibration.json"
VISION_CALIB = "sqdict.json"
OVERRIDES_FILE = "overrides.json"
STOCKFISH_PATH = r"stockfish/stockfish-windows-2022-x86-64-avx2.exe"
CAMERA_INDEX = 0 

# --- ROBOT MATH HELPERS ---
def get_xy_from_angles(b, s, e, w):
    """Calculates Gripper Tip X,Y from angles (Forward Kinematics)."""
    # MUST MATCH arm.py DIMENSIONS
    L1 = 7.0; L2 = 17.7; L3 = 19.5; L4 = 17.0
    
    rad_b = math.radians(b) 
    rad_s = math.radians(s)
    
    # Global Angles
    global_e = s - (180 - e)
    rad_e = math.radians(global_e)
    global_w = global_e - w
    rad_w = math.radians(global_w)

    # Horizontal Reach
    r = (L2 * math.cos(rad_s)) + (L3 * math.cos(rad_e)) + (L4 * math.cos(rad_w))
    
    # Cartesian Coords
    x = r * math.cos(rad_b)
    y = r * math.sin(rad_b)
    return x, y

class BoardMapper:
    def __init__(self, cal_data):
        self.a1 = cal_data['a1']
        self.h8 = cal_data['h8']
        self.x_step = (self.h8[0] - self.a1[0]) / 7.0
        self.y_step = (self.h8[1] - self.a1[1]) / 7.0
        
        # Load Overrides (Point-to-Point Pins)
        self.overrides = {}
        if os.path.exists(OVERRIDES_FILE):
            with open(OVERRIDES_FILE, 'r') as f:
                self.overrides = json.load(f)
        
    def get_coords(self, square_name):
        # 1. CHECK OVERRIDE
        if square_name in self.overrides:
            print(f"[MAP] Using Override for {square_name}")
            return self.overrides[square_name]

        # 2. USE MATH
        col = ord(square_name[0]) - ord('a')
        row = int(square_name[1]) - 1
        x = self.a1[0] + (col * self.x_step)
        y = self.a1[1] + (row * self.y_step)
        return x, y

def manual_jog(robot, b, s, e, w, g):
    # Terminal-based jog (Fallback if no GUI)
    print("  [JOG] W/S:Shoulder, A/D:Base, I/K:Elbow, J/L:Wrist. 'save' to done.")
    step = 2
    while True:
        cmd = input("  Jog > ").strip().lower()
        if cmd == 'save': return b, s, e, w, g
        
        if cmd == 'w': s += step
        if cmd == 's': s -= step
        if cmd == 'a': b += step
        if cmd == 'd': b -= step
        if cmd == 'i': e += step
        if cmd == 'k': e -= step
        if cmd == 'j': w -= step
        if cmd == 'l': w += step
        if cmd == 'o': g = 0
        if cmd == 'p': g = 180
        
        s = max(0, min(145, s))
        e = max(0, min(180, e))
        w = max(0, min(180, w))
        robot.send_cmd(b, s, e, w, g)

# --- VISION HELPERS ---
def interpolate_grid(tl, tr, br, bl):
    dict_sq = {}
    files = "abcdefgh"
    ranks = "87654321" 
    src_pts = np.array([tl, tr, br, bl], dtype="float32")
    dst_size = 800
    dst_pts = np.array([[0, 0], [dst_size, 0], [dst_size, dst_size], [0, dst_size]], dtype="float32")
    M = cv2.getPerspectiveTransform(dst_pts, src_pts)
    step = dst_size / 8.0

    for r_idx, rank in enumerate(ranks):     
        for f_idx, file in enumerate(files): 
            x1, y1 = f_idx * step, r_idx * step
            x2, y2 = (f_idx + 1) * step, (r_idx + 1) * step
            corners_logical = np.array([[[x1, y1]], [[x2, y1]], [[x2, y2]], [[x1, y2]]], dtype="float32")
            corners_camera = cv2.perspectiveTransform(corners_logical, M)
            poly = [[int(c[0][0]), int(c[0][1])] for c in corners_camera]
            dict_sq[file + rank] = poly
    return dict_sq

def detect_move(frame1, frame2, zones):
    diff = cv2.absdiff(frame1, frame2)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)
    changed = []
    
    for sq_name, poly in zones.items():
        mask = np.zeros(thresh.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [np.array(poly, dtype=np.int32)], 255)
        score = cv2.countNonZero(cv2.bitwise_and(thresh, thresh, mask=mask))
        if score > 150: changed.append((sq_name, score))
            
    changed.sort(key=lambda x: x[1], reverse=True)
    return [x[0] for x in changed]

# --- MODES (Terminal Functions) ---
def mode_cam_calib():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened(): return print("Camera not found.")
    
    pts = []
    def click(event, x, y, f, p):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4: pts.append([x, y])

    cv2.namedWindow("Camera"); cv2.setMouseCallback("Camera", click)
    print("Click corners: A8 -> H8 -> H1 -> A1. Press 'q' to save.")

    while True:
        ret, frame = cap.read()
        if not ret: break
        for i, p in enumerate(pts): cv2.circle(frame, (p[0],p[1]), 5, (0,0,255), -1)
        if len(pts) == 4: cv2.polylines(frame, [np.array(pts)], True, (255,0,0), 2)
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) == ord('q') and len(pts)==4: break
    
    cap.release(); cv2.destroyAllWindows()
    if len(pts) == 4:
        data = interpolate_grid(pts[0], pts[1], pts[2], pts[3])
        with open(VISION_CALIB, "w") as f: json.dump(data, f)
        print("Camera Calibration Saved.")

def mode_play_game(robot):
    if not os.path.exists(ROBOT_CALIB) or not os.path.exists(VISION_CALIB):
        return print("Calibrate Robot AND Camera first.")
    
    with open(ROBOT_CALIB, 'r') as f: r_mapper = BoardMapper(json.load(f))
    with open(VISION_CALIB, 'r') as f: v_zones = json.load(f)
    
    engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH_PATH)
    cap = cv2.VideoCapture(CAMERA_INDEX)
    board = chess.Board()
    
    print("\n--- VISION CHESS ---")
    print("YOU are WHITE. Robot is BLACK.")
    print("Move piece -> Remove Hand -> Press SPACEBAR.")
    
    time.sleep(1)
    ret, frame_base = cap.read()
    
    while not board.is_game_over():
        # HUMAN TURN
        print("\n[WHITE] Your Move...")
        while True:
            ret, frame = cap.read()
            vis = frame.copy()
            for sq, poly in v_zones.items():
                cv2.polylines(vis, [np.array(poly, np.int32)], True, (0,255,0), 1)
            cv2.imshow("Game", vis)
            
            if cv2.waitKey(1) == 32: # SPACE
                print("Scanning...")
                changes = detect_move(frame_base, frame, v_zones)
                print(f"Changes: {changes}")
                
                user_move = None
                for m in board.legal_moves:
                    if chess.square_name(m.from_square) in changes and \
                       chess.square_name(m.to_square) in changes:
                        user_move = m; break
                
                if user_move:
                    print(f"Detected: {user_move.uci()}")
                    board.push(user_move)
                    frame_base = frame.copy()
                    break
                else: print("Move unclear. Try again.")

        if board.is_game_over(): break

        # ROBOT TURN
        print("[BLACK] Robot Thinking...")
        result = engine.play(board, chess.engine.Limit(time=0.8))
        print(f"Robot Plays: {result.move.uci()}")
        
        sq1 = chess.square_name(result.move.from_square)
        sq2 = chess.square_name(result.move.to_square)
        x1, y1 = r_mapper.get_coords(sq1)
        x2, y2 = r_mapper.get_coords(sq2)
        
        if board.is_capture(result.move):
            robot.release(); robot.move_to_xyz(x2, y2)
            time.sleep(0.5); robot.grasp()
            robot.send_cmd(180, 90, 90, 90, 180) # Dump
            time.sleep(1); robot.release()

        robot.release(); robot.move_to_xyz(x1, y1)
        time.sleep(0.8); robot.grasp(); robot.move_to_xyz(x1, y1)
        time.sleep(0.5); robot.send_cmd(90, 110, 160, 45, 180) # Lift
        time.sleep(0.5); robot.move_to_xyz(x2, y2)
        time.sleep(0.8); robot.release(); robot.move_to_xyz(x2, y2)
        time.sleep(0.5); robot.home()
        
        board.push(result.move)
        time.sleep(1.5)
        ret, frame_base = cap.read()

    print("Game Over")
    cap.release(); cv2.destroyAllWindows(); engine.quit()

if __name__ == "__main__":
    # If run directly, show menu. Usually run via GUI though.
    robot = ChessRobotArm(port='COM8')
    while True:
        print("\n1. Cam Calib  2. Play Game  3. Exit")
        c = input("> ")
        if c=='1': mode_cam_calib()
        elif c=='2': mode_play_game(robot)
        elif c=='3': break