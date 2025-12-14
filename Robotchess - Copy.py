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
            if sq in self.ovr:
                val = self.ovr[sq]
                print(f"DEBUG: {sq} -> FOUND IN OVERRIDES -> {val}")
                
                # If the pinned value has 3 numbers (x, y, z), return all 3
                if len(val) == 3: return val[0], val[1], val[2]
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

def run_vision_game(robot, mapper):
    """Main Game Loop with Vision."""
    if not os.path.exists(VISION_FILE): return print("Run Vision Calib first!")
    with open(VISION_FILE, 'r') as f: v_zones = json.load(f)
    
    engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH_PATH)
    cap = cv2.VideoCapture(CAMERA_INDEX)
    board = chess.Board()
    
    print("\n--- GAME STARTED ---")
    print("You are WHITE. Robot is BLACK.")
    print("Move a piece, remove your hand, then press SPACEBAR.")
    
    time.sleep(1); ret, frame_base = cap.read() # Initial Snapshot
    
    while not board.is_game_over():
        # --- HUMAN TURN ---
        print("\n[WHITE] Waiting for move...")
        user_move = None
        while not user_move:
            ret, frame = cap.read()
            vis = frame.copy()
            # Draw grid for feedback
            for sq, poly in v_zones.items(): cv2.polylines(vis, [np.array(poly, np.int32)], True, (0,255,0), 1)
            cv2.imshow("Chess Vision", vis)
            
            # Press SPACE to confirm move
            if cv2.waitKey(1) == 32: 
                changes = detect_move(frame_base, frame, v_zones)
                print(f"Detected Changes: {changes}")
                
                # Deduce move from changes
                for m in board.legal_moves:
                    if chess.square_name(m.from_square) in changes and chess.square_name(m.to_square) in changes:
                        user_move = m; break
                
                if user_move: print(f"Move Detected: {user_move.uci()}")
                else: print("Move unclear. Try again.")

        board.push(user_move)
        frame_base = frame.copy() # Update base frame
        if board.is_game_over(): break

        # --- ROBOT TURN ---
        result = engine.play(board, chess.engine.Limit(time=0.8))
        print(f"[BLACK] Robot plays: {result.move.uci()}")
        
        sq1 = chess.square_name(result.move.from_square)
        sq2 = chess.square_name(result.move.to_square)
        
        # Get Coordinates (Logic + Z height)
        x1, y1, z1 = mapper.get_coords(sq1)
        x2, y2, z2 = mapper.get_coords(sq2)
        if z1 is None: z1 = Z_GRIP # Fallback if get_coords didn't calc Z
        if z2 is None: z2 = Z_GRIP

        # Execute Capture (Dump piece)
        if board.is_capture(result.move):
            robot.release(); robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(0.5)
            robot.move_to_xyz(x2, y2, z=z2); time.sleep(0.5)
            robot.grasp(); time.sleep(0.5)
            robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(0.5)
            robot.send_cmd(180, 90, 90, 90, 180) # Dump Bin
            time.sleep(1.0); robot.release(); time.sleep(0.5)

        # Execute Move (Pick & Place)
        robot.release(); robot.move_to_xyz(x1, y1, z=Z_TRAVEL); time.sleep(1.0)
        robot.move_to_xyz(x1, y1, z=z1); time.sleep(0.5); robot.grasp(); time.sleep(0.5)
        robot.move_to_xyz(x1, y1, z=Z_TRAVEL); time.sleep(0.5)
        robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(1.0)
        robot.move_to_xyz(x2, y2, z=z2); time.sleep(0.5); robot.release(); time.sleep(0.5)
        robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(0.5); robot.home()
        
        board.push(result.move)
        time.sleep(1.5); ret, frame_base = cap.read() # Update frame after robot moves away

    cap.release(); cv2.destroyAllWindows(); engine.quit()
    # <--- THIS IS THE END OF THE LOOP (Indentation moves back)
    
    # --- CHANGE THIS SECTION ---
    print("Game Over. Parking Robot...")
    robot.park()  # <--- INSERT THIS LINE
    
    cap.release(); cv2.destroyAllWindows(); engine.quit()
    
# ==================================================================================
# SECTION 5: GUI (CONTROL PANEL)
# ==================================================================================
class RobotGUI:
    def __init__(self, root):
        self.root = root; self.root.title("Chess Robot Monolith v8.0 (Complete)")
        self.root.geometry("600x950")
        self.robot = ChessRobotArm(); self.mapper = BoardMapper(); self.setup_ui()
        self.b, self.s, self.e, self.w, self.g = 90, 90, 180, 18, 34
        # --- FIX: LOAD EXISTING CALIBRATION INTO MEMORY ---
        self.calib_pts = {}
        if os.path.exists(CALIB_FILE):
            try:
                self.calib_pts = json.load(open(CALIB_FILE))
                self.log(f"Loaded {len(self.calib_pts)} points from file.")
            except:
                self.log("Calibration file corrupted or empty.")
        
        self.log(f"System Ready. Base Offset: {OFFSET_BASE}")
        
    def log(self, msg):
        self.txt.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {msg}\n"); self.txt.see(tk.END)

    def setup_ui(self):
        tabs = ttk.Notebook(self.root)
        t_jog = ttk.Frame(tabs); tabs.add(t_jog, text='1. Jog & Calib')
        t_test = ttk.Frame(tabs); tabs.add(t_test, text='2. Test & Fix')
        t_game = ttk.Frame(tabs); tabs.add(t_game, text='3. Vision Game')
        tabs.pack(expand=1, fill="both")

        # --- JOG TAB ---
        f_ctl = ttk.LabelFrame(t_jog, text="Manual Controls"); f_ctl.pack(pady=10)
        self.step = tk.IntVar(value=2)
        tk.Scale(f_ctl, from_=1, to=10, var=self.step, orient='h').pack()
        
        btns = [('W (Shld Up)',0,1,'w'), ('A (Base L)',1,0,'a'), ('S (Shld Dn)',1,1,'s'), ('D (Base R)',1,2,'d'),
                ('I (Elbow)',3,1,'i'), ('J (Extend)',4,0,'j'), ('K (Elbow)',4,1,'k'), ('L (Curl)',4,2,'l'),
                ('Open',5,0,'o'), ('Close',5,2,'p')]
        bf = ttk.Frame(f_ctl); bf.pack(pady=10)
        for t,r,c,k in btns: ttk.Button(bf, text=t, width=10, command=lambda k=k: self.jog(k)).grid(row=r, column=c)

        f_cal = ttk.LabelFrame(t_jog, text="4-Point Calibration"); f_cal.pack(pady=10, fill='x')
        ttk.Button(f_cal, text="1. Reset / Home", command=self.reset).pack(fill='x')
        ttk.Button(f_cal, text="2. Save A1 (Bottom Left)", command=lambda: self.save_pt("a1")).pack(fill='x')
        ttk.Button(f_cal, text="3. Save H1 (Bottom Right)", command=lambda: self.save_pt("h1")).pack(fill='x')
        ttk.Button(f_cal, text="4. Save H8 (Top Right)", command=lambda: self.save_pt("h8")).pack(fill='x')
        ttk.Button(f_cal, text="5. Save A8 (Top Left)", command=lambda: self.save_pt("a8")).pack(fill='x')
        ttk.Button(f_cal, text="6. Save Calibration File", command=self.save_file).pack(fill='x')

        # --- TEST TAB ---
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

        # --- GAME TAB ---
        f_vis = ttk.Frame(t_game); f_vis.pack(pady=10)
        ttk.Button(f_vis, text="1. Calibrate Camera (Click 4 Corners)", command=lambda: threading.Thread(target=run_vision_calib).start()).pack(pady=5, fill='x')
        ttk.Button(f_vis, text="2. Start Chess Game vs Robot", command=lambda: threading.Thread(target=run_vision_game, args=(self.robot, self.mapper)).start()).pack(pady=5, fill='x')

        self.txt = tk.Text(self.root, height=10); self.txt.pack(fill='both')

    # --- LOGIC METHODS ---
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

    def reset(self):
        self.calib_pts = {}; self.b, self.s, self.e, self.w, self.g = 90, 90, 180, 18, 34
        self.robot.home(); self.log("Reset.")

    def save_pt(self, n):
        # Only save X,Y,Z geometry.
        x, y, z = get_xy_from_angles(self.b, self.s, self.e, self.w)
        self.calib_pts[n] = [x, y] # For map, we only need X/Y
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
        
        # Get coordinates
        res = self.mapper.get_coords(sq)
        x, y = res[0], res[1]
        
        # LOGIC FIX:
        # If res[2] (Z) exists, it means it's either Pinned OR Auto-Calculated by get_coords.
        # In both cases, it is the FINAL PHYSICAL Z. We trust it.
        target_z = res[2] if (len(res) > 2 and res[2] is not None) else Z_GRIP
        
        # Just pass the final Z. IK will no longer mess with it.
        self.robot.gripper_state = 180
        self.robot.move_to_xyz(x, y, z=Z_TRAVEL); time.sleep(0.5)
        self.robot.move_to_xyz(x, y, z=target_z)

    
        # Sync GUI
        angles = self.robot.inverse_kinematics(x, y, z_height=target_z)
        if angles: 
            self.b, self.s, self.e, self.w = angles; self.g=180
            msg = f"Arrived {sq}. Z={target_z:.1f}"
            if len(res)>2 and res[2]: msg += " (PINNED)"
            self.log(msg)

    def pin_sq(self):
        sq = self.ent_sq.get().strip().lower()
        x, y, z = get_xy_from_angles(self.b, self.s, self.e, self.w)
        d = {}
        if os.path.exists(OVERRIDES_FILE): d = json.load(open(OVERRIDES_FILE))
        d[sq] = [x, y, z] # Save Z!
        json.dump(d, open(OVERRIDES_FILE, 'w')); self.mapper = BoardMapper(); self.log(f"Pinned {sq} at Z={z:.1f}")

    def unpin_sq(self):
        sq = self.ent_sq.get().strip().lower()
        d = json.load(open(OVERRIDES_FILE))
        if sq in d: del d[sq]; json.dump(d, open(OVERRIDES_FILE, 'w')); self.mapper = BoardMapper(); self.log(f"Unpinned {sq}")

    def run_sim(self): threading.Thread(target=self._sim).start()
    def _sim(self):
        m = self.ent_mv.get().strip().lower(); cap = self.chk_cap.get()
        # Helper to get safe Z for simulator
        def get_z(s):
            r = self.mapper.get_coords(s)
            # If we have a Pinned Z (override), use it
            if len(r) > 2 and r[2]: return r[2]
            
            # CALCULATE DISTANCE
            dist = math.sqrt(r[0]**2 + r[1]**2)
            
            # NEW FORMULA: Curve + Slope + Intercept
            correction = (dist**2 * TILT_CURVE) + (dist * TILT_SLOPE) + TILT_INTERCEPT
            
            return Z_GRIP + max(MAX_TILT_DROP, correction)
        x1, y1, _ = self.mapper.get_coords(m[0:2]); z1 = get_z(m[0:2])
        x2, y2, _ = self.mapper.get_coords(m[2:4]); z2 = get_z(m[2:4])
        
        if cap:
            self.robot.release(); self.robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(1)
            self.robot.move_to_xyz(x2, y2, z=z2); time.sleep(0.5); self.robot.grasp(); time.sleep(0.5)
            self.robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(0.5)
            self.robot.send_cmd(180, 90, 90, 90, 180); time.sleep(1); self.robot.release(); time.sleep(0.5)
        self.robot.release(); self.robot.move_to_xyz(x1, y1, z=Z_TRAVEL); time.sleep(1)
        self.robot.move_to_xyz(x1, y1, z=z1); time.sleep(0.5); self.robot.grasp(); time.sleep(0.5)
        self.robot.move_to_xyz(x1, y1, z=Z_TRAVEL); time.sleep(0.5); self.robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(1)
        self.robot.move_to_xyz(x2, y2, z=z2); time.sleep(0.5); self.robot.release(); time.sleep(0.5)
        self.robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(0.5); self.robot.home()

if __name__ == "__main__":
    root = tk.Tk(); app = RobotGUI(root); root.mainloop()