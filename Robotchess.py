"""
==================================================================================
CHESS ROBOT MONOLITH v8.0 (Complete System)
==================================================================================
This single script controls the entire project:
1.  HARDWARE: talks to the Arduino (Servo control).
2.  MATH: Calculates angles (Inverse Kinematics).
3.  VISION: Uses the camera to see the board (OpenCV).
4.  LOGIC: Plays chess against you (Stockfish).
5.  GUI: A control panel to calibrate and test everything.

FOR NOVICES:
- Sections are clearly marked.
- Change numbers in the "CONFIGURATION" section to tune your robot.
- Do not edit the "HARDWARE DRIVER" unless you know Python well.
"""

import tkinter as tk
from tkinter import ttk
import cv2              # Library for Computer Vision (seeing the board)
import numpy as np      # Library for advanced math/matrix calculations
import json             # Library to save/load text files (.json)
import os               # Library to check if files exist on your PC
import time             # Library to add delays (sleep)
import threading        # Library to run background tasks (so GUI doesn't freeze)
import math             # Library for trigonometry (sin, cos, atan2)
import serial           # Library to talk to Arduino via USB
import chess            # Library to track the rules of chess
import chess.engine     # Library to talk to the AI (Stockfish)
from tkinter import messagebox
# ==================================================================================
# SECTION 1: CONFIGURATION (TUNE YOUR ROBOT HERE)
# ==================================================================================

# --- FILE PATHS ---
# Where to save your settings. 'r' means "raw string" (handles Windows backslashes).
CALIB_FILE = "calibration.json"       # Stores the 4 corners of the board
VISION_FILE = "vision_grid.json"      # Stores where squares are in the CAMERA view
OVERRIDES_FILE = "overrides.json"     # Stores your manual "Pinned" positions
# UPDATE THIS PATH to point to your Stockfish .exe file!
STOCKFISH_PATH = r"stockfish/stockfish-windows-2022-x86-64-avx2.exe"
CAMERA_INDEX = 0                      # 0 is usually the default webcam. Try 1 if 0 fails.

# --- ROBOT DIMENSIONS (cm) ---
# Measure your physical arm parts with a ruler. Accuracy is critical!
L_BASE_H = 7.0     # Height from table to Shoulder pivot
L_SHOULDER = 18.0  # Length of Upper Arm 17.7
L_ELBOW = 20.0     # Length of Forearm 19.5
L_WRIST = 17.0     # Length from Wrist pivot to Gripper Tip

# --- OFFSETS (CORRECTIONS) ---
# Use these to fix physical misalignments.
# OFFSET_BASE: If robot points too Left/Right when centered.
OFFSET_BASE = 16.0     #13.5 
OFFSET_SHOULDER = 4.0  # Lifts arm slightly to fight gravity "sag"
OFFSET_ELBOW = 0.0
OFFSET_WRIST = 0.0

# --- HEIGHT STRATEGY ---
# --- HEIGHT STRATEGY ---
Z_TRAVEL = 8.0  
Z_GRIP = 2.0            # WAS 5.0. Drops everything by 3.0 cm.


# --- TILT CORRECTION (FINAL PRECISION) ---
TILT_CURVE = 0.0035     # Slightly flatter bowl
TILT_SLOPE = 0.04       # Positive slope lifts the front/middle
TILT_INTERCEPT = -3.0   # Base anchor
MAX_TILT_DROP = -10.0

# --- SAFETY LIMITS ---
# Prevents the servos from smashing into the robot's own frame.
SHLD_MIN = 0; SHLD_MAX = 175  # 145 Shoulder Range (0 to 165 degrees)
ELBW_MIN = 0; ELBW_MAX = 180  # Elbow Range
WRST_MIN = 0; WRST_MAX = 180  # Wrist Range
MIN_SAFE_SHOULDER_ANGLE = 55.0 # Don't let the shoulder lean too far forward!
# --- SERVO PULSE MAP ---
SERVO_MIN_US = 400
SERVO_MAX_US = 2400
SERVO_RANGE_DEG = 180.0

# ==================================================================================
# SECTION 2: HARDWARE DRIVER (THE MUSCLE)
# ==================================================================================
class ChessRobotArm:
    """Controls the physical motors via Arduino."""
    
    def __init__(self, port='COM8'):
        self.connected = False
        try:
            self.ser = serial.Serial(port, 9600, timeout=1)
            print(f"[HW] Connected to Robot on {port}")
            self.connected = True
            
            # --- UPDATE: Wait for Startup Sequence ---
            # Arduino moves Park -> Home (approx 4 seconds)
            print("[HW] Waiting 5s for robot to unfold...")
            time.sleep(5) 
            
        except Exception as e:
            print(f"[HW] Simulation Mode (No Serial): {e}")

        self.gripper_state = self.deg_to_us(34) 

    def deg_to_us(self, deg):
        d = max(0, min(180, deg))
        return int((d / 180.0) * (2400 - 400) + 400)

    def send_cmd(self, b, s, e, w, g):
        if not self.connected: return
        
        # 1. Add Tuning Offsets
        b_phys = b + OFFSET_BASE
        s_phys = s + OFFSET_SHOULDER
        e_phys = e + OFFSET_ELBOW
        w_phys = w + OFFSET_WRIST
        
        # 2. Clamp angles
        b_final = max(0, min(180, b_phys))
        s_final = max(SHLD_MIN, min(SHLD_MAX, s_phys))
        e_final = max(ELBW_MIN, min(ELBW_MAX, e_phys))
        w_final = max(WRST_MIN, min(WRST_MAX, w_phys))
        
        msg = f"{self.deg_to_us(b_final)},{self.deg_to_us(s_final)},{self.deg_to_us(e_final)},{self.deg_to_us(w_final)},{self.deg_to_us(g)}\n"
        self.ser.write(msg.encode()) 
        
        while True:
            if self.ser.in_waiting and self.ser.readline().decode().strip() == "done": break

    # --- NEW: PARKING FUNCTION ---
    def park(self):
        """Sends raw pulses to move to the safe folded position."""
        if not self.connected: return
        print("[HW] Parking Robot...")
        # Base 90, Shoulder 70, Elbow 80, Wrist 80, Grip 34 (In Pulses)
        msg = "1550,1400,1289,1289,778\n"
        self.ser.write(msg.encode())
        
        while True:
            if self.ser.in_waiting and self.ser.readline().decode().strip() == "done": break

    def inverse_kinematics(self, x, y, z_height=Z_TRAVEL):
        # REMOVED "+ 90" because your X-axis is Left/Right (0-180)
        base_angle = math.atan2(y, x) * 180 / math.pi
        
        # If your servo logic requires positive angles only, ensuring it's 0-180:
        # (This handles the case if atan2 returns negative for right side)
        if base_angle < 0: base_angle += 360 # normalize
        r_total = math.sqrt(x**2 + y**2)
        z_final = z_height
        
        best_sol = None
        for pitch in range(-90, 5, 5): 
            rad_p = math.radians(pitch)
            dr = L_WRIST * math.cos(rad_p); dz = L_WRIST * math.sin(rad_p)
            r_w = r_total - dr; z_w = (z_final - dz) - L_BASE_H 
            h = math.sqrt(r_w**2 + z_w**2)
            if h > (L_SHOULDER + L_ELBOW): continue 
            try:
                alpha = math.acos((L_SHOULDER**2 + h**2 - L_ELBOW**2) / (2 * L_SHOULDER * h))
                beta = math.acos((L_SHOULDER**2 + L_ELBOW**2 - h**2) / (2 * L_SHOULDER * L_ELBOW))
            except ValueError: continue 
            theta = math.atan2(z_w, r_w)
            s = (theta + alpha) * 180 / math.pi; e = beta * 180 / math.pi
            w = (s - (180 - e)) - pitch 
            if s > SHLD_MAX: continue 
            if s >= MIN_SAFE_SHOULDER_ANGLE: return base_angle, s, e, w
            if best_sol is None or s > best_sol[1]: best_sol = (base_angle, s, e, w)
        return best_sol 

    def move_to_xyz(self, x, y, z=Z_TRAVEL):
        angles = self.inverse_kinematics(x, y, z_height=z)
        if angles:
            self.send_cmd(*angles, self.gripper_state)
            return True
        return False
    
    def grasp(self): self.gripper_state = 180; pass
    def release(self): self.gripper_state = 34; pass # Open 
    def home(self): self.send_cmd(90, 90, 180, 18, 34) # Ready Pose

# ==================================================================================
# SECTION 3: MATH HELPERS (GEOMETRY & MAPPING)
# ==================================================================================
def get_xy_from_angles(b, s, e, w):
    """
    Forward Kinematics: Calculates X/Y position from current angles.
    Used during calibration to "Record" where the robot is.
    """
    rb, rs = math.radians(b), math.radians(s)
    ge = s - (180 - e); re = math.radians(ge)
    gw = ge - w; rw = math.radians(gw)
    
    r = (L_SHOULDER*math.cos(rs)) + (L_ELBOW*math.cos(re)) + (L_WRIST*math.cos(rw))
    z = L_BASE_H + (L_SHOULDER*math.sin(rs)) + (L_ELBOW*math.sin(re)) + (L_WRIST*math.sin(rw))
    
    return r * math.cos(rb), r * math.sin(rb), z

class BoardMapper:
    def __init__(self):
        # 1. LOAD VISION CALIBRATION (Pixels)
        # This tells the code where squares are in the CAMERA image.
        try:
            with open("sqdict.json", "r") as f:
                sq_data = json.load(f)
            
            # Helper to get center of a square in PIXELS
            def get_px_center(k):
                pts = sq_data[k]
                return [sum(p[0] for p in pts)/4, sum(p[1] for p in pts)/4]

            # Use accurate pixel centers for the vision matrix
            self.vision_cal = {
                "a1": get_px_center("a1"), # ~ [519, 123]
                "h1": get_px_center("h1"), # ~ [244, 118]
                "h8": get_px_center("h8"), # ~ [237, 400]
                "a8": get_px_center("a8")  # ~ [518, 405]
            }
        except:
            print("ERROR: Could not load sqdict.json!")
            self.vision_cal = {"a1":[0,0], "h1":[100,0], "h8":[100,100], "a8":[0,100]}
        
        # --- DEBUG OVERRIDES LOADING ---
        self.ovr = {}
        if os.path.exists("overrides.json"):
            try:
                with open("overrides.json", "r") as f:
                    self.ovr = json.load(f)
                print(f"SYSTEM: Loaded {len(self.ovr)} overrides: {list(self.ovr.keys())}")
            except Exception as e:
                print(f"CRITICAL ERROR: Could not load overrides.json! Reason: {e}")
        else:
            print("SYSTEM: overrides.json file NOT FOUND.")
            
        # 2. LOAD PHYSICAL CALIBRATION (Centimeters)
        # This tells the code where squares are in the REAL WORLD.
        self.phys_cal = {}
        if os.path.exists("calibration.json"):
             with open("calibration.json", "r") as f:
                 self.phys_cal = json.load(f)

        # 3. SETUP VISION MATRIX (Logical Grid -> Camera Pixels)
        src = np.array([[0,0], [7,0], [7,7], [0,7]], dtype="float32") 
        dst = np.array([
            self.vision_cal["a1"], 
            self.vision_cal["h1"], 
            self.vision_cal["h8"], 
            self.vision_cal["a8"]
        ], dtype="float32")
        self.matrix = cv2.getPerspectiveTransform(src, dst)
        
    def get_coords(self, sq):
            sq = sq.strip().lower()
            
            # PRIORITY 1: CHECK OVERRIDES (THE PINNED SQUARES)
            # This MUST be first. If we pinned it, we go exactly there.
            #if sq in self.ovr:
            #    val = self.ovr[sq]
            #    print(f"DEBUG: {sq} -> FOUND IN OVERRIDES -> {val}")
            #    
                # If the pinned value has 3 numbers (x, y, z), return all 3
            #    if len(val) == 3: return val[0], val[1], val[2]
            #    return val[0], val[1], None
            
            # PRIORITY 1: CHECK OVERRIDES
            if sq in self.ovr:
                val = self.ovr[sq]
                
                # Case A: Old format or just XYZ (Length 3)
                if len(val) == 3: return val[0], val[1], val[2]
                
                # Case B: New "Angle" format (Length > 3)
                # We still return X,Y,Z so the interpolator works
                if len(val) > 3: return val[0], val[1], val[2]
                
                return val[0], val[1], None
            
            
            
            # PRIORITY 2: CHECK PHYSICAL CALIBRATION (THE 4 CORNERS)
            # If it's A1, H1, A8, or H8, use the jogged corner directly.
            if sq in self.phys_cal:
                x, y = self.phys_cal[sq]
                
                # Calculate Auto-Z since corners usually don't have pinned Z
                dist = math.sqrt(x**2 + y**2)
                z = max(MAX_TILT_DROP, (dist**2 * TILT_CURVE) + (dist * TILT_SLOPE) + TILT_INTERCEPT) + Z_GRIP
                
                print(f"DEBUG: {sq} -> FOUND IN CALIBRATION -> CM({x:.1f}, {y:.1f})")
                return x, y, z

            # PRIORITY 3: INTERPOLATE (THE MATH)
            # Used for all unpinned middle squares.
            col = ord(sq[0]) - ord('a') # 0-7
            row = int(sq[1]) - 1        # 0-7
            
            # Get corners from physical calibration
            xa1, ya1 = self.phys_cal.get("a1", [0,0])
            xa8, ya8 = self.phys_cal.get("a8", [0,1])
            xh1, yh1 = self.phys_cal.get("h1", [1,0])
            xh8, yh8 = self.phys_cal.get("h8", [1,1])

            # Bilinear Interpolation Formula
            u = col / 7.0
            v = row / 7.0
            
            x = (1-u)*(1-v)*xa1 + u*(1-v)*xh1 + u*v*xh8 + (1-u)*v*xa8
            y = (1-u)*(1-v)*ya1 + u*(1-v)*yh1 + u*v*yh8 + (1-u)*v*ya8
            
            # Calculate Auto-Z
            dist = math.sqrt(x**2 + y**2)
            z = max(MAX_TILT_DROP, (dist**2 * TILT_CURVE) + (dist * TILT_SLOPE) + TILT_INTERCEPT) + Z_GRIP
            
            print(f"DEBUG: {sq} -> Interpolated -> CM({x:.1f}, {y:.1f})")
            return x, y, z
# ==================================================================================
# SECTION 4: COMPUTER VISION (EYES)
# ==================================================================================
def interpolate_grid(tl, tr, br, bl):
    """
    Divides the camera image into 64 squares based on 4 clicked corners.
    Returns a dictionary of polygons: {'a1': [[x,y],...], 'a2': ...}
    """
    dict_sq = {}
    files = "abcdefgh"; ranks = "87654321" 
    src = np.array([tl, tr, br, bl], dtype="float32")
    dst = np.array([[0,0], [800,0], [800,800], [0,800]], dtype="float32")
    
    # Calculate Perspective Matrix for Camera View
    M = cv2.getPerspectiveTransform(dst, src)
    step = 100.0 # 800px / 8 squares

    for r_idx, rank in enumerate(ranks):     
        for f_idx, file in enumerate(files): 
            # Define square in logical space
            x1, y1 = f_idx * step, r_idx * step
            pts = np.array([[[x1,y1]], [[x1+step,y1]], [[x1+step,y1+step]], [[x1,y1+step]]], dtype="float32")
            
            # Transform to Camera Pixel space
            cam_pts = cv2.perspectiveTransform(pts, M)
            dict_sq[file + rank] = [[int(c[0][0]), int(c[0][1])] for c in cam_pts]
    return dict_sq

def detect_move(frame1, frame2, zones):
    """
    Compares two camera frames to find changes.
    Returns the square name with the most pixel changes (e.g., 'e2').
    """
    # 1. Calculate Difference
    diff = cv2.absdiff(frame1, frame2)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)
    
    changed = []
    # 2. Check each square zone
    for sq, poly in zones.items():
        mask = np.zeros(thresh.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [np.array(poly, dtype=np.int32)], 255)
        score = cv2.countNonZero(cv2.bitwise_and(thresh, thresh, mask=mask))
        if score > 150: changed.append((sq, score)) # Threshold score
            
    changed.sort(key=lambda x: x[1], reverse=True) # Sort by most changed
    return [x[0] for x in changed] # Return names like ['e2', 'e4']

def run_vision_calib():
    """Opens camera and lets user click 4 corners to calibrate vision."""
    cap = cv2.VideoCapture(CAMERA_INDEX)
    pts = []
    
    def click_event(e, x, y, f, p): 
        if e == cv2.EVENT_LBUTTONDOWN and len(pts) < 4: pts.append([x, y])
    
    cv2.namedWindow("Calibrate Camera"); cv2.setMouseCallback("Calibrate Camera", click_event)
    print("VISION CALIB: Click corners: Top-Left (A8) -> Top-Right (H8) -> Bottom-Right (H1) -> Bottom-Left (A1)")

    while True:
        ret, frame = cap.read()
        if not ret: break
        # Draw clicked points
        for i, p in enumerate(pts): cv2.circle(frame, (p[0],p[1]), 5, (0,0,255), -1)
        if len(pts) == 4: cv2.polylines(frame, [np.array(pts)], True, (255,0,0), 2)
        
        cv2.imshow("Calibrate Camera", frame)
        if cv2.waitKey(1) == ord('q') and len(pts)==4: break # Press Q to save
    
    cap.release(); cv2.destroyAllWindows()
    if len(pts) == 4:
        grid_data = interpolate_grid(pts[0], pts[1], pts[2], pts[3])
        with open(VISION_FILE, "w") as f: json.dump(grid_data, f)
        print("Vision Calibration Saved.")

def run_vision_game(robot, mapper, is_white, app):
    """
    Unified Game Loop with EMERGENCY STOP.
    Arguments:
      app: The main GUI object (to check app.stop_requested)
    """
    # 1. SETUP IDENTITY
    robot_color_name = "WHITE" if is_white else "BLACK"
    human_color_name = "BLACK" if is_white else "WHITE"
    
    v_file = f"vision_{robot_color_name.lower()}.json"
    if not os.path.exists(v_file):
        print(f"Warning: {v_file} not found. Using default...")
        v_file = "vision_grid.json"
        
    if not os.path.exists(v_file): return print("CRITICAL: No calibration file found.")
    
    with open(v_file, 'r') as f: v_zones = json.load(f)
    print(f"--- GAME STARTING ({robot_color_name}) ---")

    # 2. INIT
    engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH_PATH)
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    time.sleep(1)
    
    board = chess.Board()
    
    # SAFETY POPUP
    ret, preview = cap.read()
    if ret: cv2.imshow("Chess Vision", preview); cv2.waitKey(1)
    
    msg = f"Robot: {robot_color_name}\n\nClick OK to Start."
    messagebox.showinfo("CONFIRM START", msg)

    # --- VARS ---
    GRIP_WIDE, GRIP_SAFE, GRIP_CLOSED = 30, 30, 105
    cur_b, cur_s, cur_e, cur_w = 90, 90, 180, 18 

    # --- MOVEMENT HELPER (WITH STOP CHECK) ---
    def raw_move(b, s, e, w, g):
        nonlocal cur_b, cur_s, cur_e, cur_w
        # Emergency Check inside movement
        if app.stop_requested: return 
        
        robot.send_cmd(b, s, e, w, g)
        cur_b, cur_s, cur_e, cur_w = b, s, e, w
        robot.gripper_state = g 
        time.sleep(0.8) 

    def land(sq, grip_angle):
        if app.stop_requested: return
        # ... (Existing land logic remains same) ...
        # (Copy your logic here or use the simplified version below)
        r = mapper.get_coords(sq)
        z = r[2] if len(r) > 2 and r[2] else Z_GRIP + max(MAX_TILT_DROP, (math.sqrt(r[0]**2 + r[1]**2)**2 * TILT_CURVE) + (math.sqrt(r[0]**2 + r[1]**2) * TILT_SLOPE) + TILT_INTERCEPT)
        angles = robot.inverse_kinematics(r[0], r[1], z_height=z)
        if angles: raw_move(angles[0], angles[1], angles[2], angles[3], grip_angle)

    # ============================
    # MAIN LOOP
    # ============================
    ret, frame_base = cap.read()
    while not ret: time.sleep(0.5); ret, frame_base = cap.read()

    while not board.is_game_over():
        
        # --- STOP CHECK ---
        if app.stop_requested:
            print("STOP TRIGGERED by User.")
            break

        is_robot_turn = (board.turn == is_white)

        if is_robot_turn:
            print(f"[{robot_color_name}] Thinking...")
            result = engine.play(board, chess.engine.Limit(time=0.8))
            
            # --- EXECUTE MOVE ---
            # (Standard move logic... shortened here for brevity, 
            #  but paste your full move logic from previous steps)
            
            # EXAMPLE OF SAFE MOVE EXECUTION:
            m_str = result.move.uci()
            sq_src, sq_dst = m_str[0:2], m_str[2:4]
            x1, y1, _ = mapper.get_coords(sq_src)
            x2, y2, _ = mapper.get_coords(sq_dst)
            
            # 1. Hover
            raw_move(cur_b, cur_s, cur_e, cur_w, GRIP_WIDE)
            robot.move_to_xyz(x1, y1, z=Z_TRAVEL); time.sleep(1)
            
            # Check Stop before diving
            if app.stop_requested: break 

            land(sq_src, GRIP_WIDE) 
            raw_move(cur_b, cur_s, cur_e, cur_w, GRIP_CLOSED)
            robot.move_to_xyz(x1, y1, z=Z_TRAVEL); time.sleep(0.5)
            
            robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(1)
            land(sq_dst, GRIP_CLOSED) 
            
            robot.send_cmd(cur_b, cur_s, cur_e, cur_w, GRIP_SAFE) # Release
            time.sleep(0.5)
            
            robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(0.5)
            robot.home(); cur_b, cur_s, cur_e, cur_w = 90, 90, 180, 18
            
            board.push(result.move)
            time.sleep(2.0)
            ret, frame_base = cap.read() # Update snapshot
            while not ret: time.sleep(0.5); ret, frame_base = cap.read()

        else:
            # --- HUMAN TURN ---
            print(f"\n[{human_color_name}] Your Turn...")
            user_move = None
            while not user_move:
                if app.stop_requested: break # Exit inner loop

                ret, frame = cap.read()
                if not ret or frame is None: time.sleep(0.2); continue

                vis = frame.copy()
                for sq, poly in v_zones.items(): cv2.polylines(vis, [np.array(poly, np.int32)], True, (0,255,0), 1)
                cv2.imshow("Chess Vision", vis)
                
                if cv2.waitKey(1) == 32: 
                    changes = detect_move(frame_base, frame, v_zones)
                    for m in board.legal_moves:
                        if chess.square_name(m.from_square) in changes and chess.square_name(m.to_square) in changes:
                            user_move = m; break
            
            if app.stop_requested: break # Exit outer loop
            board.push(user_move)
            frame_base = frame.copy()

    # --- CLEANUP (Run when loop breaks) ---
    cap.release()
    cv2.destroyAllWindows()
    engine.quit()
    print("Game Stopped. Parking Robot...")
    robot.park()
    messagebox.showinfo("GAME OVER", "The game was stopped or finished.")
    
# ==================================================================================
# SECTION 5: GUI (CONTROL PANEL) - UPDATED FOR BUTTON-BASED VISION
# ==================================================================================
class RobotGUI:
    def __init__(self, root):
        self.root = root; self.root.title("Chess Robot Monolith v8.0 (Complete)")
        self.root.geometry("600x950")
        self.robot = ChessRobotArm(); self.mapper = BoardMapper(); self.setup_ui()
        self.b, self.s, self.e, self.w, self.g = 90, 90, 180, 18, 34
        
        # Physical Calibration Data
        self.calib_pts = {}
        if os.path.exists(CALIB_FILE):
            try: self.calib_pts = json.load(open(CALIB_FILE))
            except: pass
            
        # Vision Calibration Data
        self.vis_running = False
        self.vis_pts = {}
        self.last_click = None
        
        self.log(f"System Ready. Base Offset: {OFFSET_BASE}")
        
    def log(self, msg):
        self.txt.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {msg}\n"); self.txt.see(tk.END)

    def setup_ui(self):
        tabs = ttk.Notebook(self.root)
        t_jog = ttk.Frame(tabs); tabs.add(t_jog, text='1. Jog & Calib')
        t_test = ttk.Frame(tabs); tabs.add(t_test, text='2. Test & Fix')
        t_game = ttk.Frame(tabs); tabs.add(t_game, text='3. Vision Game')
        tabs.pack(expand=1, fill="both")

        # --- TAB 1: JOG & PHYSICAL CALIBRATION ---
        f_ctl = ttk.LabelFrame(t_jog, text="Manual Controls"); f_ctl.pack(pady=10)
        self.step = tk.IntVar(value=2)
        tk.Scale(f_ctl, from_=1, to=10, var=self.step, orient='h').pack()
        
        btns = [('W (Shld Up)',0,1,'w'), ('A (Base L)',1,0,'a'), ('S (Shld Dn)',1,1,'s'), ('D (Base R)',1,2,'d'),
                ('I (Elbow)',3,1,'i'), ('J (Extend)',4,0,'j'), ('K (Elbow)',4,1,'k'), ('L (Curl)',4,2,'l'),
                ('Open',5,0,'o'), ('Close',5,2,'p')]
        bf = ttk.Frame(f_ctl); bf.pack(pady=10)
        for t,r,c,k in btns: ttk.Button(bf, text=t, width=10, command=lambda k=k: self.jog(k)).grid(row=r, column=c)

        f_cal = ttk.LabelFrame(t_jog, text="4-Point Physical Calibration"); f_cal.pack(pady=10, fill='x')
        
        # 1. The Safe "Home" Button (Kept inside the normal menu)
        ttk.Button(f_cal, text="1. Move Home (Keep Calib)", command=self.move_home).pack(fill='x', pady=2)
        
        # (The Standard Save Buttons)
        ttk.Button(f_cal, text="2. Save A1 (Bottom Left)", command=lambda: self.save_pt("a1")).pack(fill='x')
        ttk.Button(f_cal, text="3. Save H1 (Bottom Right)", command=lambda: self.save_pt("h1")).pack(fill='x')
        ttk.Button(f_cal, text="4. Save H8 (Top Right)", command=lambda: self.save_pt("h8")).pack(fill='x')
        ttk.Button(f_cal, text="5. Save A8 (Top Left)", command=lambda: self.save_pt("a8")).pack(fill='x')
        ttk.Button(f_cal, text="6. Save Calibration File", command=self.save_file).pack(fill='x')

        # --- DANGER ZONE (Moved to the very bottom) ---
        # We use tk.Button (not ttk) so we can make it RED
        lbl_spacer = ttk.Label(t_jog, text="-- DANGER ZONE --")
        lbl_spacer.pack(side="bottom", pady=(0, 5))
        
        btn_reset = tk.Button(t_jog, text="⚠ DELETE CALIBRATION DATA ⚠", 
                              bg="red", fg="white", font=("Arial", 10, "bold"),
                              command=self.reset_calibration)
        # 'side=bottom' forces it to the footer of the tab, 'pady=20' adds a safety gap
        btn_reset.pack(side="bottom", fill="x", pady=20, padx=10)
        
        # --- TAB 2: TEST & FIX ---
        f_tst = ttk.Frame(t_test); f_tst.pack(pady=10)
        self.ent_sq = ttk.Entry(f_tst); self.ent_sq.pack()
        ttk.Button(f_tst, text="Go To Square", command=self.goto_sq).pack(pady=5)
        ttk.Button(f_tst, text="Override (Pin XYZ)", command=self.pin_sq).pack(fill='x')
        ttk.Button(f_tst, text="Reset Pin", command=self.unpin_sq).pack(fill='x')
        
        ttk.Label(f_tst, text="-- SIMULATOR --").pack(pady=10)
        self.ent_mv = ttk.Entry(f_tst); self.ent_mv.pack()
        self.chk_cap = tk.BooleanVar()
        ttk.Checkbutton(f_tst, text="Capture?", var=self.chk_cap).pack()
        ttk.Button(f_tst, text="Execute Move", command=self.run_sim).pack()

        # --- TAB 3: VISION GAME ---
        f_vis = ttk.LabelFrame(t_game, text="Vision Calibration"); f_vis.pack(pady=10, fill='x')
        
        # Step 1: Open Camera
        ttk.Button(f_vis, text="1. Start Camera Feed", command=self.start_vision_stream).pack(fill='x', pady=5)
        
        # Step 2: The 4 Buttons (Grid Layout)
        f_grid = ttk.Frame(f_vis); f_grid.pack(pady=5)
        ttk.Button(f_grid, text="2. Set A8", width=15, command=lambda: self.save_vis_pt("a8")).grid(row=0, column=0, padx=5, pady=2)
        ttk.Button(f_grid, text="3. Set H8", width=15, command=lambda: self.save_vis_pt("h8")).grid(row=0, column=1, padx=5, pady=2)
        ttk.Button(f_grid, text="4. Set A1", width=15, command=lambda: self.save_vis_pt("a1")).grid(row=1, column=0, padx=5, pady=2)
        ttk.Button(f_grid, text="5. Set H1", width=15, command=lambda: self.save_vis_pt("h1")).grid(row=1, column=1, padx=5, pady=2)
        
        # --- NEW SECTION: CALIBRATION SIDE SELECTION (MISSING IN YOUR FILE) ---
        ttk.Label(f_vis, text="Save Calibration For (Robot's View):", font=("Arial", 10, "bold")).pack(pady=(10, 5))
        self.calib_side_var = tk.BooleanVar(value=True) # True=White, False=Black
        
        row_calib = ttk.Frame(f_vis)
        row_calib.pack(pady=5)
        ttk.Radiobutton(row_calib, text="White Perspective", variable=self.calib_side_var, value=True).pack(side="left", padx=10)
        ttk.Radiobutton(row_calib, text="Black Perspective", variable=self.calib_side_var, value=False).pack(side="left", padx=10)
        # -----------------------------------------------------------------------

        # Step 3: Save
        ttk.Button(f_vis, text="6. Save & Close Camera", command=self.stop_vision_stream).pack(fill='x', pady=5)

        # --- ROBOT SIDE SELECTION ---
        self.robot_color_var = tk.BooleanVar(value=True) 
        ttk.Label(t_game, text="Select Robot Side:", font=("Arial", 10, "bold")).pack(pady=(15, 5))
        
        frame_radio = ttk.Frame(t_game)
        frame_radio.pack()
        ttk.Radiobutton(frame_radio, text="Robot is WHITE (Moves First)", variable=self.robot_color_var, value=True).pack(anchor="w")
        ttk.Radiobutton(frame_radio, text="Robot is BLACK (Waits)", variable=self.robot_color_var, value=False).pack(anchor="w")
        
        # --- START BUTTON ---
        ttk.Button(t_game, text="START CHESS GAME", command=self.start_game_thread).pack(pady=10, fill='x', ipady=5)
        
        # --- NEW: STOP BUTTON (MISSING IN YOUR FILE) ---
        btn_stop = tk.Button(t_game, text="STOP GAME & PARK", bg="#ffcccc", fg="red", font=("Arial", 10, "bold"),
                             command=self.stop_game_gracefully)
        btn_stop.pack(pady=5, fill='x', ipady=5)
        # -----------------------------------------------

        self.txt = tk.Text(self.root, height=10); self.txt.pack(fill='both')
        
    # ---------------------------------------------------------
    # NEW: STOP COMMAND
    # ---------------------------------------------------------
    def stop_game_gracefully(self):
        """Stops the game if running, or just parks if idle."""
        self.stop_requested = True
        self.log("STOP REQUESTED...")

        # Check if the game thread exists and is actually alive
        game_is_running = hasattr(self, 'game_thread') and self.game_thread.is_alive()

        if game_is_running:
            # If game is running, let the game loop handle the parking (to avoid collisions)
            self.log("Game is running. Waiting for loop to finish and park...")
        else:
            # If NO game is running, park immediately!
            self.log("Game not active. Parking immediately.")
            self.robot.park()
            
       
    # --- VISION METHODS (NEW) ---
    def start_vision_stream(self):
        if self.vis_running: return
        self.vis_running = True
        self.vis_pts = {} # Reset
        self.last_click = None
        threading.Thread(target=self._vision_loop, daemon=True).start()
        self.log("Camera Opened. Click on the video, then click a button to save.")

    def _vision_loop(self):
        cap = cv2.VideoCapture(CAMERA_INDEX)
        cv2.namedWindow("Vision Setup")
        
        def mouse_cb(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                self.last_click = [x, y]
                print(f"Click at: {x}, {y}") # Console debug
        
        cv2.setMouseCallback("Vision Setup", mouse_cb)
        
        while self.vis_running:
            ret, frame = cap.read()
            if not ret: break
            
            # Draw Crosshair on last click
            if self.last_click:
                cx, cy = self.last_click
                cv2.drawMarker(frame, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

            # Draw Saved Points
            for sq, pt in self.vis_pts.items():
                cv2.circle(frame, (pt[0], pt[1]), 5, (0, 0, 255), -1)
                cv2.putText(frame, sq.upper(), (pt[0]+10, pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Vision Setup", frame)
            if cv2.waitKey(1) == ord('q'): break # Emergency exit
        
        cap.release()
        cv2.destroyAllWindows()
        self.vis_running = False

    def save_vis_pt(self, sq):
        if not self.vis_running:
            return self.log("Error: Start Camera first!")
        if not self.last_click:
            return self.log("Error: Click a point on the video first!")
        
        self.vis_pts[sq] = self.last_click
        self.log(f"Saved Vision Point {sq.upper()} at {self.last_click}")

    # ---------------------------------------------------------
    # UPDATED: SAVE CALIBRATION
    # ---------------------------------------------------------
    def stop_vision_stream(self):
        self.vis_running = False
        
        # 1. Check if we have all 4 points
        required = ['a8', 'h8', 'h1', 'a1']
        if any(k not in self.vis_pts for k in required):
            self.log("Error: You must click ALL 4 buttons (A8, H8, A1, H1) before saving.")
            cv2.destroyAllWindows() # Safe to close here if we failed early
            return

        # 2. Calculate the Grid Zones (The Logic Fix)
        # We pass the points in the strict order: A8, H8, H1, A1
        # This ensures the math works even if A8 is at the bottom-right of your screen!
        self.v_zones = interpolate_grid(
            self.vis_pts['a8'], 
            self.vis_pts['h8'], 
            self.vis_pts['h1'], 
            self.vis_pts['a1']
        )

        # 3. Save to File
        is_white_calib = self.calib_side_var.get()
        filename = "vision_white.json" if is_white_calib else "vision_black.json"
        
        try:
            with open(filename, 'w') as f:
                json.dump(self.v_zones, f)
            self.log(f"Calibration saved to {filename}")
            
            # Backup as default
            with open("vision_grid.json", 'w') as f: json.dump(self.v_zones, f)
        except Exception as e:
            self.log(f"Error saving: {e}")

        # Let the thread close the window naturally, don't force it here
        
    def start_game_thread(self):
        if self.vis_running:
            self.log("Error: Please 'Save & Close' camera first.")
            return
        
        # Reset the stop flag before starting
        self.stop_requested = False
        
        is_white = self.robot_color_var.get()
        
        # --- MODIFIED: Save the thread to a variable so we can check it later ---
        self.game_thread = threading.Thread(target=run_vision_game, args=(self.robot, self.mapper, is_white, self))
        self.game_thread.start()
        
    # --- LOGIC METHODS (EXISTING) ---
    def jog(self, k):
        s = self.step.get()
        if k=='w': self.s+=s; 
        if k=='s': self.s-=s
        if k=='a': self.b+=s; 
        if k=='d': self.b-=s
        if k=='i': self.e+=s; 
        if k=='k': self.e-=s
        if k=='j': self.w-=s; 
        if k=='l': self.w+=s
        if k=='o': self.g=34; 
        if k=='p': self.g=180
        self.s = max(SHLD_MIN, min(SHLD_MAX, self.s))
        self.e = max(ELBW_MIN, min(ELBW_MAX, self.e))
        self.w = max(WRST_MIN, min(WRST_MAX, self.w))
        self.robot.send_cmd(self.b, self.s, self.e, self.w, self.g)
        self.log(f"Pos: B{self.b} S{self.s} E{self.e} W{self.w}")

    def move_home(self):
        """Moves the robot to home position safely without deleting data."""
        self.b, self.s, self.e, self.w, self.g = 90, 90, 180, 18, 34
        self.robot.home()
        self.log("Moved Home (Calibration Preserved).")

    def reset_calibration(self):
        """Only deletes the calibration points from memory."""
        self.calib_pts = {}
        self.log("Calibration Data DELETED. Please re-calibrate 4 points.")
        
    def save_pt(self, n):
        x, y, z = get_xy_from_angles(self.b, self.s, self.e, self.w)
        self.calib_pts[n] = [x, y]
        self.log(f"Saved {n} at {x:.1f}, {y:.1f}")

    def save_file(self):
        req = ["a1", "h1", "h8", "a8"]
        for r in req:
            if r not in self.calib_pts: return self.log(f"Err: Missing {r}")
        json.dump(self.calib_pts, open(CALIB_FILE, 'w'))
        self.mapper = BoardMapper() # Reload
        self.log("Calibration Saved!")

    def goto_sq(self):
        sq = self.ent_sq.get().strip().lower()
        self.mapper = BoardMapper()
        
        is_angle_mode = False
        saved_angles = []
        if sq in self.mapper.ovr:
            val = self.mapper.ovr[sq]
            if len(val) > 5 and val[7] == "ANGLES":
                is_angle_mode = True
                saved_angles = val[3:7]

        res = self.mapper.get_coords(sq)
        x, y = res[0], res[1]
        target_z = res[2] if (len(res) > 2 and res[2] is not None) else Z_GRIP

        self.robot.gripper_state = 180
        self.robot.move_to_xyz(x, y, z=Z_TRAVEL)
        time.sleep(0.5)
        
        if is_angle_mode:
            b, s, e, w = saved_angles
            self.robot.send_cmd(b, s, e, w, 180)
            self.b, self.s, self.e, self.w = b, s, e, w
            self.g = 180
            self.log(f"Arrived {sq} using EXACT SAVED ANGLES.")
        else:
            self.robot.move_to_xyz(x, y, z=target_z)
            angles = self.robot.inverse_kinematics(x, y, z_height=target_z)
            if angles: 
                self.b, self.s, self.e, self.w = angles
                self.g = 180
                self.log(f"Arrived {sq}. Z={target_z:.1f}")

    def pin_sq(self):
        sq = self.ent_sq.get().strip().lower()
        x, y, z = get_xy_from_angles(self.b, self.s, self.e, self.w)
        saved_data = [x, y, z, self.b, self.s, self.e, self.w, "ANGLES"]
        
        d = {}
        if os.path.exists(OVERRIDES_FILE): 
            try: d = json.load(open(OVERRIDES_FILE))
            except: pass
        d[sq] = saved_data
        json.dump(d, open(OVERRIDES_FILE, 'w'))
        self.mapper = BoardMapper()
        self.log(f"Pinned {sq} (CAPTURED EXACT ANGLES)")

    def unpin_sq(self):
        sq = self.ent_sq.get().strip().lower()
        d = json.load(open(OVERRIDES_FILE))
        if sq in d: del d[sq]; json.dump(d, open(OVERRIDES_FILE, 'w')); self.mapper = BoardMapper(); self.log(f"Unpinned {sq}")

    def run_sim(self): threading.Thread(target=self._sim).start()
    def _sim(self):
        m = self.ent_mv.get().strip().lower()
        raw_cap = self.chk_cap.get(); cap = (raw_cap == 1) 
        
        GRIP_WIDE, GRIP_SAFE, GRIP_CLOSED = 30, 30, 105

        def raw_move(b, s, e, w, g):
            self.robot.send_cmd(b, s, e, w, g)
            self.b, self.s, self.e, self.w, self.g = b, s, e, w, g
            self.robot.gripper_state = g 
            time.sleep(0.8) 

        def land(sq, grip_angle):
            if sq in self.mapper.ovr:
                val = self.mapper.ovr[sq]
                if len(val) > 7 and val[7] == "ANGLES":
                    raw_move(val[3], val[4], val[5], val[6], grip_angle)
                    return
            r = self.mapper.get_coords(sq)
            z = r[2] if len(r)>2 and r[2] else Z_GRIP + max(MAX_TILT_DROP, (math.sqrt(r[0]**2+r[1]**2)**2 * TILT_CURVE) + (math.sqrt(r[0]**2+r[1]**2) * TILT_SLOPE) + TILT_INTERCEPT)
            angles = self.robot.inverse_kinematics(r[0], r[1], z_height=z)
            if angles: raw_move(angles[0], angles[1], angles[2], angles[3], grip_angle)

        if len(m) < 4: return
        x1, y1, _ = self.mapper.get_coords(m[0:2])
        x2, y2, _ = self.mapper.get_coords(m[2:4])

        if cap:
            raw_move(self.b, self.s, self.e, self.w, GRIP_WIDE)
            self.robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(1)
            land(m[2:4], GRIP_WIDE) 
            raw_move(self.b, self.s, self.e, self.w, GRIP_CLOSED)
            self.robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(0.5)
            raw_move(180, 90, 90, 90, GRIP_CLOSED); time.sleep(0.5)
            raw_move(180, 90, 90, 90, GRIP_WIDE); time.sleep(0.5)

        raw_move(self.b, self.s, self.e, self.w, GRIP_WIDE)
        self.robot.move_to_xyz(x1, y1, z=Z_TRAVEL); time.sleep(1)
        land(m[0:2], GRIP_WIDE)
        raw_move(self.b, self.s, self.e, self.w, GRIP_CLOSED)
        self.robot.move_to_xyz(x1, y1, z=Z_TRAVEL); time.sleep(0.5)
        self.robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(1)
        land(m[2:4], GRIP_CLOSED) 
        
        target_b, target_s, target_e, target_w = self.b, self.s, self.e, self.w
        if m[2:4] in self.mapper.ovr: target_b, target_s, target_e, target_w = self.mapper.ovr[m[2:4]][3:7]
        self.robot.send_cmd(target_b, target_s, target_e, target_w, GRIP_SAFE)
        self.robot.gripper_state = GRIP_SAFE; self.g = GRIP_SAFE
        time.sleep(1.0) 
        
        self.robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(0.5)
        self.robot.home(); self.b, self.s, self.e, self.w = 0, 90, 90, 90
        
# ==================================================================================
# MAIN EXECUTION
# ==================================================================================
if __name__ == "__main__":
    print("Launching Chess Robot Monolith...")
    
    # Check if critical files exist
    if not os.path.exists(STOCKFISH_PATH):
        print(f"WARNING: Stockfish not found at {STOCKFISH_PATH}")
    
    # Start the GUI
    root = tk.Tk()
    
    # Protect against closing the window aggressively
    def on_closing():
        print("Closing application...")
        root.destroy()
        os._exit(0) # Force kill threads
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    app = RobotGUI(root)
    root.mainloop()