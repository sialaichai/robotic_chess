import cv2
import numpy as np
import json
import os
import time
import math
import chess
import chess.engine
from arm import ChessRobotArm, Z_TRAVEL, Z_GRIP

# --- CONFIG ---
ROBOT_CALIB = "calibration.json"
VISION_CALIB = "sqdict.json"
OVERRIDES_FILE = "overrides.json"
STOCKFISH_PATH = r"stockfish/stockfish-windows-2022-x86-64-avx2.exe"
CAMERA_INDEX = 0 

# --- ROBOT HELPERS (SYNCED) ---
def get_xy_from_angles(b, s, e, w):
    L1 = 7.0; L2 = 17.7; L3 = 19.5; L4 = 17.0
    OFFSET_BASE = 0.0      # REMOVED
    OFFSET_SHOULDER = 4.0  # KEPT
    
    true_b = b + OFFSET_BASE
    true_s = s + OFFSET_SHOULDER
    rad_b, rad_s = math.radians(true_b), math.radians(true_s)
    global_e = true_s - (180 - e); rad_e = math.radians(global_e)
    global_w = global_e - w; rad_w = math.radians(global_w)
    r = (L2 * math.cos(rad_s)) + (L3 * math.cos(rad_e)) + (L4 * math.cos(rad_w))
    return r * math.cos(rad_b), r * math.sin(rad_b)

class BoardMapper:
    def __init__(self, cal_data):
        self.a1 = cal_data['a1']; self.h8 = cal_data['h8']
        self.x_step = (self.h8[0] - self.a1[0]) / 7.0
        self.y_step = (self.h8[1] - self.a1[1]) / 7.0
        self.overrides = {}
        if os.path.exists(OVERRIDES_FILE):
            with open(OVERRIDES_FILE, 'r') as f: self.overrides = json.load(f)
    def get_coords(self, sq):
        if sq in self.overrides: return self.overrides[sq]
        c = ord(sq[0]) - ord('a'); row = int(sq[1]) - 1
        return self.a1[0] + (c * self.x_step), self.a1[1] + (row * self.y_step)

# --- VISION ---
def interpolate_grid(tl, tr, br, bl):
    dict_sq = {}; files = "abcdefgh"; ranks = "87654321" 
    src = np.array([tl, tr, br, bl], dtype="float32")
    dst = np.array([[0,0], [800,0], [800,800], [0,800]], dtype="float32")
    M = cv2.getPerspectiveTransform(dst, src); step = 100.0
    for r, rank in enumerate(ranks):     
        for f, file in enumerate(files): 
            x1, y1 = f*step, r*step
            pts = np.array([[[x1,y1]], [[x1+step,y1]], [[x1+step,y1+step]], [[x1,y1+step]]], dtype="float32")
            cam_pts = cv2.perspectiveTransform(pts, M)
            dict_sq[file+rank] = [[int(c[0][0]), int(c[0][1])] for c in cam_pts]
    return dict_sq

def detect_move(frame1, frame2, zones):
    diff = cv2.absdiff(frame1, frame2)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)
    changed = []
    for sq, poly in zones.items():
        mask = np.zeros(thresh.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [np.array(poly, dtype=np.int32)], 255)
        score = cv2.countNonZero(cv2.bitwise_and(thresh, thresh, mask=mask))
        if score > 150: changed.append((sq, score))
    changed.sort(key=lambda x: x[1], reverse=True)
    return [x[0] for x in changed]

# --- MODES ---
def mode_cam_calib():
    cap = cv2.VideoCapture(CAMERA_INDEX); pts = []
    def click(e, x, y, f, p): 
        if e == cv2.EVENT_LBUTTONDOWN and len(pts) < 4: pts.append([x, y])
    cv2.namedWindow("Cam"); cv2.setMouseCallback("Cam", click)
    while True:
        ret, frame = cap.read()
        for p in pts: cv2.circle(frame, (p[0],p[1]), 5, (0,0,255), -1)
        cv2.imshow("Cam", frame)
        if cv2.waitKey(1) == ord('q') and len(pts)==4: break
    cap.release(); cv2.destroyAllWindows()
    if len(pts) == 4: json.dump(interpolate_grid(*pts), open(VISION_CALIB, "w"))

def mode_play_game(robot):
    if not os.path.exists(ROBOT_CALIB) or not os.path.exists(VISION_CALIB): return
    with open(ROBOT_CALIB, 'r') as f: r_map = BoardMapper(json.load(f))
    with open(VISION_CALIB, 'r') as f: v_zones = json.load(f)
    engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH_PATH)
    cap = cv2.VideoCapture(CAMERA_INDEX); board = chess.Board()
    time.sleep(1); ret, f_base = cap.read()
    
    while not board.is_game_over():
        print("\n[WHITE] Your Move..."); user_move = None
        while not user_move:
            ret, frame = cap.read(); cv2.imshow("Game", frame)
            if cv2.waitKey(1) == 32: # SPACE
                changes = detect_move(f_base, frame, v_zones)
                for m in board.legal_moves:
                    if chess.square_name(m.from_square) in changes and chess.square_name(m.to_square) in changes:
                        user_move = m; break
        board.push(user_move); f_base = frame.copy(); print(board)
        if board.is_game_over(): break

        res = engine.play(board, chess.engine.Limit(time=0.8)); print(f"[BLACK] Robot: {res.move.uci()}")
        x1, y1 = r_map.get_coords(chess.square_name(res.move.from_square))
        x2, y2 = r_map.get_coords(chess.square_name(res.move.to_square))
        
        if board.is_capture(res.move):
            robot.release(); robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(0.5)
            robot.move_to_xyz(x2, y2, z=Z_GRIP); time.sleep(0.5); robot.grasp(); time.sleep(0.5)
            robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(0.5)
            robot.send_cmd(180, 90, 90, 90, 180); time.sleep(1); robot.release(); time.sleep(0.5)

        robot.release(); robot.move_to_xyz(x1, y1, z=Z_TRAVEL); time.sleep(1.0)
        robot.move_to_xyz(x1, y1, z=Z_GRIP); time.sleep(0.5); robot.grasp(); time.sleep(0.5)
        robot.move_to_xyz(x1, y1, z=Z_TRAVEL); time.sleep(0.5)
        robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(1.0)
        robot.move_to_xyz(x2, y2, z=Z_GRIP); time.sleep(0.5); robot.release(); time.sleep(0.5)
        robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(0.5); robot.home()
        
        board.push(res.move); print(board); time.sleep(1.5); ret, f_base = cap.read()
    cap.release(); cv2.destroyAllWindows(); engine.quit()