"""
==================================================================================
CHESS ROBOT MONOLITH v8.20 (Fixing The "Silent Crash" in Grip)
==================================================================================
1. FIXED: 'triple_grab' had a typo (self.s vs self.cur_s) that caused a crash.
2. FIXED: Simplified the grab sequence to be robust.
3. DEBUG: Added print confirmation of Grip Angle sent to hardware.
"""

import tkinter as tk
from tkinter import ttk
import cv2              
import numpy as np      
import json             
import os               
import time             
import threading        
import math             
import serial           
import chess            
import chess.engine     
from tkinter import messagebox

# ==================================================================================
# SECTION 1: CONFIGURATION
# ==================================================================================

STOCKFISH_PATH = r"stockfish/stockfish-windows-2022-x86-64-avx2.exe"
CAMERA_INDEX = 0                      

# --- ROBOT DIMENSIONS (cm) ---
L_BASE_H = 7.0     
L_SHOULDER = 18.0  
L_ELBOW = 20.0     
L_WRIST = 17.0     

# --- OFFSETS ---
OFFSET_BASE = 16.0     
OFFSET_SHOULDER = 4.0  
OFFSET_ELBOW = 0.0
OFFSET_WRIST = 0.0

# --- HEIGHT STRATEGY ---
Z_TRAVEL = 8.0  
Z_GRIP = 2            
Z_HOVER = 14.0          

# --- GRIPPER ---
GRIP_OPEN_ANGLE = 35    
GRIP_CLOSED_ANGLE = 145 

# --- TILT CORRECTION ---
TILT_CURVE = 0.0035     
TILT_SLOPE = 0.17   
TILT_INTERCEPT = -5
MAX_TILT_DROP = -10.0

# --- SAFETY LIMITS ---
SHLD_MIN = 0; SHLD_MAX = 175  
ELBW_MIN = 0; ELBW_MAX = 180  
WRST_MIN = 0; WRST_MAX = 180  
MIN_SAFE_SHOULDER_ANGLE = 55.0 

# ==================================================================================
# SECTION 2: HARDWARE DRIVER
# ==================================================================================
class ChessRobotArm:
    def __init__(self, port='COM8'):
        self.connected = False
        try:
            self.ser = serial.Serial(port, 9600, timeout=1)
            print(f"[HW] Connected on {port}")
            self.connected = True
            time.sleep(5) 
        except Exception as e:
            print(f"[HW] Simulation Mode: {e}")

        self.gripper_state = self.deg_to_us(GRIP_OPEN_ANGLE) 

    def deg_to_us(self, deg):
        d = max(0, min(180, deg))
        return int((d / 180.0) * (2400 - 400) + 400)

    def send_cmd(self, b, s, e, w, g):
        if not self.connected: return
        b_final = max(0, min(180, b + OFFSET_BASE))
        s_final = max(SHLD_MIN, min(SHLD_MAX, s + OFFSET_SHOULDER))
        e_final = max(ELBW_MIN, min(ELBW_MAX, e + OFFSET_ELBOW))
        w_final = max(WRST_MIN, min(WRST_MAX, w + OFFSET_WRIST))
        # --- ENABLED DEBUGGING HERE ---
        # This will show you: G150 (Tight) vs G145 (Loose)
        print(f"[HW] B{b_final:.0f} S{s_final:.0f} E{e_final:.0f} W{w_final:.0f} G{g}")
        # Debug print to confirm gripper is receiving the right value
        # print(f"[DEBUG] Sending: B{b:.1f} S{s:.1f} E{e:.1f} W{w:.1f} G{g}")
        
        msg = f"{self.deg_to_us(b_final)},{self.deg_to_us(s_final)},{self.deg_to_us(e_final)},{self.deg_to_us(w_final)},{self.deg_to_us(g)}\n"
        self.ser.write(msg.encode()) 
        while True:
            if self.ser.in_waiting and self.ser.readline().decode().strip() == "done": break

    def park(self):
        if not self.connected: return
        print("[HW] Parking...")
        msg = f"1550,1400,1289,1289,{self.deg_to_us(GRIP_CLOSED_ANGLE)}\n"
        self.ser.write(msg.encode())
        while True:
            if self.ser.in_waiting and self.ser.readline().decode().strip() == "done": break

    def inverse_kinematics(self, x, y, z_height=Z_TRAVEL):
        base_angle = math.atan2(y, x) * 180 / math.pi
        if base_angle < 0: base_angle += 360 
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
    
    def home(self): self.send_cmd(90, 90, 180, 18, GRIP_OPEN_ANGLE) 

# ==================================================================================
# SECTION 3: MATH HELPERS & THE UNIFIED "ROBOT MOTION" BRAIN
# ==================================================================================
def get_xy_from_angles(b, s, e, w):
    rb, rs = math.radians(b), math.radians(s)
    ge = s - (180 - e); re = math.radians(ge)
    gw = ge - w; rw = math.radians(gw)
    r = (L_SHOULDER*math.cos(rs)) + (L_ELBOW*math.cos(re)) + (L_WRIST*math.cos(rw))
    z = L_BASE_H + (L_SHOULDER*math.sin(rs)) + (L_ELBOW*math.sin(re)) + (L_WRIST*math.sin(rw))
    return r * math.cos(rb), r * math.sin(rb), z

class BoardMapper:
    def __init__(self, is_white_profile=True):
        suffix = "white" if is_white_profile else "black"
        self.cal_file = f"calibration_{suffix}.json"
        self.ovr_file = f"overrides_{suffix}.json"
        
        print(f"[MAPPER] Loading Profile: {suffix.upper()}")

        self.ovr = {}
        if os.path.exists(self.ovr_file):
            try: self.ovr = json.load(open(self.ovr_file, "r"))
            except: pass
            
        self.phys_cal = {}
        if os.path.exists(self.cal_file):
             with open(self.cal_file, "r") as f: self.phys_cal = json.load(f)
        else:
             print(f"[WARN] No {self.cal_file} found. Defaulting to empty.")

    def get_coords(self, sq):
            sq = sq.strip().lower()
            if sq in self.ovr:
                val = self.ovr[sq]
                if len(val) >= 3: return val[0], val[1], val[2]
            
            if sq in self.phys_cal:
                x, y = self.phys_cal[sq]
                dist = math.sqrt(x**2 + y**2)
                z = max(MAX_TILT_DROP, (dist**2 * TILT_CURVE) + (dist * TILT_SLOPE) + TILT_INTERCEPT) + Z_GRIP
                return x, y, z

            col = ord(sq[0]) - ord('a'); row = int(sq[1]) - 1        
            xa1, ya1 = self.phys_cal.get("a1", [0,0]); xa8, ya8 = self.phys_cal.get("a8", [0,1])
            xh1, yh1 = self.phys_cal.get("h1", [1,0]); xh8, yh8 = self.phys_cal.get("h8", [1,1])

            u = col / 7.0; v = row / 7.0
            x = (1-u)*(1-v)*xa1 + u*(1-v)*xh1 + u*v*xh8 + (1-u)*v*xa8
            y = (1-u)*(1-v)*ya1 + u*(1-v)*yh1 + u*v*yh8 + (1-u)*v*ya8
            dist = math.sqrt(x**2 + y**2)
            z = max(MAX_TILT_DROP, (dist**2 * TILT_CURVE) + (dist * TILT_SLOPE) + TILT_INTERCEPT) + Z_GRIP
            return x, y, z

# --- THE UNIFIED BRAIN CLASS ---
# --- THE UNIFIED BRAIN CLASS (v8.21: Anti-Stall & Grip Redundancy) ---
# --- THE SAFETY PATH CLASS (v8.22: Linear Travel Fix) ---
# --- THE UNIFIED BRAIN CLASS (v8.23: Final Grip & Motion Fix) ---
# --- THE UNIFIED BRAIN CLASS (v8.24: Max Force & Double Bite) ---
# --- THE UNIFIED BRAIN CLASS (v9.0: Stateful "Sticky" Grip) ---
# --- THE UNIFIED BRAIN CLASS (v9.0: Stateful "Sticky" Grip) ---
# This version REMOVES grip arguments from movement functions.
# It uses a global memory (self.current_grip) so the robot NEVER forgets to crush.
class RobotMotion:
    def __init__(self, robot, mapper, stop_check_callback):
        self.robot = robot
        self.mapper = mapper
        self.check_stop = stop_check_callback
        
        # ROBOT MEMORY
        self.cur_b, self.cur_s, self.cur_e, self.cur_w = 90, 90, 180, 18
        
        # --- THE FIX: SINGLE SOURCE OF TRUTH ---
        # We store the desired grip here. 
        # All movements AUTOMATICALLY use this angle.
        self.current_grip = GRIP_OPEN_ANGLE 
        
        # MAX CRUSH FORCE (Stall the servo to hold tight)
        self.GRIP_MAX_FORCE = 180 

    def safe_wait(self, duration):
        if self.check_stop(): return
        time.sleep(duration)

    # --- NEW: EXPLICIT GRIP COMMANDS ---
    def lock_hand(self):
        """Closes the hand to MAX force and remembers it."""
        if self.check_stop(): return
        print("[MOTION] Locking Hand (MAX FORCE)...")
        self.current_grip = self.GRIP_MAX_FORCE # Update Memory
        # Send command immediately
        self.robot.send_cmd(self.cur_b, self.cur_s, self.cur_e, self.cur_w, self.current_grip)
        self.robot.gripper_state = self.current_grip
        time.sleep(0.8) # Wait for servo to crush

    def open_hand(self):
        """Opens the hand and remembers it."""
        print("[MOTION] Opening Hand...")
        self.current_grip = GRIP_OPEN_ANGLE # Update Memory
        
        # Jiggle open (helps release sticky pieces)
        self.robot.send_cmd(self.cur_b, self.cur_s, self.cur_e, self.cur_w, GRIP_OPEN_ANGLE - 10)
        time.sleep(0.1)
        self.robot.send_cmd(self.cur_b, self.cur_s, self.cur_e, self.cur_w, self.current_grip)
        self.robot.gripper_state = self.current_grip
        time.sleep(0.3)

    # --- MOVEMENT PRIMITIVES (They now just obey self.current_grip) ---
    def raw_move(self, b, s, e, w):
        if self.check_stop(): return
        self.cur_b, self.cur_s, self.cur_e, self.cur_w = b, s, e, w
        # ALWAYS use the remembered grip
        self.robot.send_cmd(b, s, e, w, self.current_grip) 
        self.safe_wait(1.5)

    def smart_move_xyz(self, x, y, z):
        if self.check_stop(): return
        angles = self.robot.inverse_kinematics(x, y, z_height=z)
        if angles:
            self.raw_move(angles[0], angles[1], angles[2], angles[3])
        else:
            print(f"[WARN] Unreachable: {x:.1f}, {y:.1f}, {z:.1f}")

    # --- COMPLEX ACTIONS ---
    def land_on_square(self, sq):
        """Descends to a square. Grip stays as it is (Open if picking, Closed if placing)."""
        if self.check_stop(): return
        
        if sq in self.mapper.ovr and len(self.mapper.ovr[sq]) > 5:
            val = self.mapper.ovr[sq]
            print(f"[MOTION] Using PINNED angles for {sq}")
            self.raw_move(val[3], val[4], val[5], val[6])
            return

        r = self.mapper.get_coords(sq)
        tx, ty = r[0], r[1]
        final_z = r[2] if len(r) > 2 and r[2] else Z_GRIP + max(MAX_TILT_DROP, (math.sqrt(tx**2 + ty**2)**2 * TILT_CURVE) + (math.sqrt(tx**2 + ty**2) * TILT_SLOPE) + TILT_INTERCEPT)
        
        start_z = Z_HOVER
        steps = 5
        for i in range(1, steps):
            inter_z = start_z + (final_z - start_z) * (i / steps)
            angles = self.robot.inverse_kinematics(tx, ty, z_height=inter_z)
            if angles:
                self.robot.send_cmd(angles[0], angles[1], angles[2], angles[3], self.current_grip)
                time.sleep(0.05)
        
        self.smart_move_xyz(tx, ty, final_z)

    def segmented_lift(self, x, y):
        """Lifts straight up. Grip stays LOCKED if we called lock_hand()."""
        if self.check_stop(): return
        target_z = Z_HOVER
        start_z = Z_GRIP 
        steps = 5
        for i in range(1, steps):
            inter_z = start_z + (target_z - start_z) * (i / steps)
            angles = self.robot.inverse_kinematics(x, y, z_height=inter_z)
            if angles:
                self.robot.send_cmd(angles[0], angles[1], angles[2], angles[3], self.current_grip)
                time.sleep(0.05)
        self.smart_move_xyz(x, y, target_z)

    def travel_straight(self, start_sq, end_sq):
        """Moves between squares. Grip stays LOCKED."""
        if self.check_stop(): return
        x1, y1, _ = self.mapper.get_coords(start_sq)
        x2, y2, _ = self.mapper.get_coords(end_sq)
        dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        steps = max(5, int(dist / 2.0))
        
        print(f"[MOTION] Pathing {start_sq}->{end_sq} (Grip: {self.current_grip})...")

        for i in range(1, steps + 1):
            if self.check_stop(): break
            t = i / steps
            curr_x = x1 + (x2 - x1) * t
            curr_y = y1 + (y2 - y1) * t
            angles = self.robot.inverse_kinematics(curr_x, curr_y, z_height=Z_HOVER)
            if angles:
                self.robot.send_cmd(angles[0], angles[1], angles[2], angles[3], self.current_grip)
                self.cur_b, self.cur_s, self.cur_e, self.cur_w = angles[0], angles[1], angles[2], angles[3]
                time.sleep(0.05) 

    # --- HIGH LEVEL GAME MOVES ---

    def execute_move(self, sq_src, sq_dst):
        print(f"[MOTION] Executing Move: {sq_src} -> {sq_dst}")
        x_src, y_src, z_src = self.mapper.get_coords(sq_src)
        x_dst, y_dst, z_dst = self.mapper.get_coords(sq_dst)

        # 1. Approach Source (Open)
        self.open_hand()
        self.smart_move_xyz(x_src, y_src, Z_HOVER)
        self.land_on_square(sq_src)
        
        # 2. Grab (Locks Memory to MAX FORCE)
        self.lock_hand()
        
        # 3. Lift & Travel (Memory stays Locked - IMPOSSIBLE TO DROP)
        self.segmented_lift(x_src, y_src)
        self.travel_straight(sq_src, sq_dst)
        
        # 4. Land & Release
        self.land_on_square(sq_dst)
        self.open_hand()
        
        # 5. Retreat
        self.segmented_lift(x_dst, y_dst)

    def execute_capture(self, sq_src, sq_dst):
        print(f"[MOTION] Executing Capture: {sq_src} x {sq_dst}")
        x_vic, y_vic, _ = self.mapper.get_coords(sq_dst)
        
        # --- PHASE 1: REMOVE VICTIM ---
        self.open_hand()
        self.smart_move_xyz(x_vic, y_vic, Z_HOVER)
        self.land_on_square(sq_dst)
        
        self.lock_hand() # Memory = 180 (Crush)
        
        self.segmented_lift(x_vic, y_vic) # Still Crushing
        
        # Trash Dump (Using your XYZ)
        trash_x, trash_y, trash_z = -15.0, 15.0, 10.0 
        self.smart_move_xyz(trash_x, trash_y, trash_z) # Still Crushing
        self.open_hand() # Release
        
        # --- PHASE 2: MOVE ATTACKER ---
        self.execute_move(sq_src, sq_dst)
        
# ==================================================================================
# SECTION 4: VISION & GAME LOOP
# ==================================================================================
def interpolate_grid(tl, tr, br, bl):
    dict_sq = {}
    files = "abcdefgh"; ranks = "87654321" 
    src = np.array([tl, tr, br, bl], dtype="float32")
    dst = np.array([[0,0], [800,0], [800,800], [0,800]], dtype="float32")
    M = cv2.getPerspectiveTransform(dst, src)
    step = 100.0 
    for r_idx, rank in enumerate(ranks):     
        for f_idx, file in enumerate(files): 
            x1, y1 = f_idx * step, r_idx * step
            pts = np.array([[[x1,y1]], [[x1+step,y1]], [[x1+step,y1+step]], [[x1,y1+step]]], dtype="float32")
            cam_pts = cv2.perspectiveTransform(pts, M)
            dict_sq[file + rank] = [[int(c[0][0]), int(c[0][1])] for c in cam_pts]
    return dict_sq

def detect_move(frame1, frame2, zones):
    f1 = cv2.GaussianBlur(frame1, (5,5), 0)
    f2 = cv2.GaussianBlur(frame2, (5,5), 0)
    diff = cv2.absdiff(f1, f2)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    
    # --- TUNING SETTINGS ---
    # 1. THRESHOLD: Lowered from 60 to 25. 
    #    This detects subtle gray changes (black piece on black square).
    _, thresh = cv2.threshold(gray, 15, 255, cv2.THRESH_BINARY)
    
    kernel = np.ones((3,3), np.uint8)
    
    # 2. EROSION: Reduced from 2 to 1.
    #    Previous setting of 2 was "eating" the faint outline of the knight.
    thresh = cv2.erode(thresh, kernel, iterations=1)
    
    changed = []
    for sq, poly in zones.items():
        mask = np.zeros(thresh.shape, dtype=np.uint8)
        cv2.fillPoly(mask, [np.array(poly, dtype=np.int32)], 255)
        score = cv2.countNonZero(cv2.bitwise_and(thresh, thresh, mask=mask))
        
        # 3. SCORE: Lowered from 300 to 150.
        #    We accept smaller blobs of change now.
        if score > 150: changed.append((sq, score)) 
        
    changed.sort(key=lambda x: x[1], reverse=True) 
    return [x[0] for x in changed]

def run_vision_game(robot, is_white, app):
    mapper = BoardMapper(is_white_profile=is_white)
    robot_color = "WHITE" if is_white else "BLACK"
    v_file = f"vision_{robot_color.lower()}.json"
    if not os.path.exists(v_file): return print(f"CRITICAL: No vision file {v_file}")
    
    with open(v_file, 'r') as f: v_zones = json.load(f)
    print(f"--- GAME STARTING ({robot_color}) ---")

    engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH_PATH)
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    time.sleep(1)
    board = chess.Board()

    # --- UNIFIED MOTION CONTROLLER ---
    motion = RobotMotion(robot, mapper, lambda: app.stop_requested)
    
    # Callback to keep window alive
    def update_window(duration):
        end = time.time() + duration
        while time.time() < end:
            ret, frame = cap.read()
            if ret: cv2.imshow("Chess Vision", frame)
            cv2.waitKey(30)
    
    motion.safe_wait = update_window

    # Initial read
    ret, frame_base = cap.read()
    while not ret: update_window(0.5); ret, frame_base = cap.read()

    while not board.is_game_over():
        if app.stop_requested: break
        if (board.turn == is_white):
            print(f"[{robot_color}] Thinking...")
            result = engine.play(board, chess.engine.Limit(time=0.8))
            
            m_str = result.move.uci()
            sq_src, sq_dst = m_str[0:2], m_str[2:4]
            
            if board.is_capture(result.move):
                motion.execute_capture(sq_src, sq_dst)
            else:
                motion.execute_move(sq_src, sq_dst)

            if app.stop_requested: break 
            
            robot.home()
            board.push(result.move)
            update_window(2.0) 
            ret, frame_base = cap.read() 
            while not ret: update_window(0.5); ret, frame_base = cap.read()

        else:
            print(f"\n[HUMAN] Your Turn...")
            # DEBUG: Print whose turn the Engine thinks it is
            eng_turn = "WHITE" if board.turn == chess.WHITE else "BLACK"
            print(f"[DEBUG] Internal Engine State: It is {eng_turn}'s turn.")
            
            user_move = None
            while not user_move:
                if app.stop_requested: break 
                ret, frame = cap.read()
                if not ret: update_window(0.2); continue
                
                vis = frame.copy()
                for sq, poly in v_zones.items(): 
                    cv2.polylines(vis, [np.array(poly, np.int32)], True, (0,255,0), 1)
                
                cv2.putText(vis, "SPACE: Submit | R: Reset Cam", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.imshow("Chess Vision", vis)
                
                key = cv2.waitKey(30)
                
                # --- MANUAL RESET (Keep this from previous step) ---
                if key == ord('r'):
                    print("[VISION] Manual Reset Requested...")
                    time.sleep(1.0)
                    ret, frame_base = cap.read()
                    print("[VISION] New Reference Image Captured!")
                    continue 

                # --- MOVE ANALYSIS ---
                if key == 32: 
                    print("[VISION] Spacebar Detected! Analyzing move...")
                    changes = detect_move(frame_base, frame, v_zones)
                    print(f"[DEBUG] Detected Changes: {changes}")
                    
                    # 1. Try to find a legal move
                    match_found = False
                    for m in board.legal_moves:
                        if chess.square_name(m.from_square) in changes and chess.square_name(m.to_square) in changes:
                            user_move = m
                            match_found = True
                            print(f"[SUCCESS] Identified Legal Move: {m.uci()}")
                            break
                    
                    # 2. If FAILED, explain WHY in detail
                    if not match_found:
                        print(f"[VISION] REJECTED. Analysis:")
                        print(f"   -> Engine says it is {eng_turn}'s turn.")
                        
                        # Check if the squares form a move that is just currently illegal
                        if len(changes) >= 2:
                            src, dst = changes[0], changes[1]
                            # Check both directions
                            try:
                                m1 = chess.Move.from_uci(f"{src}{dst}")
                                m2 = chess.Move.from_uci(f"{dst}{src}")
                                if m1 in board.legal_moves: print(f"   -> WEIRD: {src}->{dst} IS legal, but logic missed it?")
                                else: print(f"   -> {src}->{dst} is ILLEGAL.")
                                
                                if m2 in board.legal_moves: print(f"   -> WEIRD: {dst}->{src} IS legal, but logic missed it?")
                                else: print(f"   -> {dst}->{src} is ILLEGAL.")
                            except:
                                pass
                                
                        if eng_turn == "WHITE" and not is_white:
                             print("   -> CRITICAL ERROR: Computer thinks it's still the Robot's turn!")
            
            if app.stop_requested: break 
            board.push(user_move); frame_base = frame.copy()
            
    
# ==================================================================================
# SECTION 5: GUI (NOW USES UNIFIED MOTION CLASS)
# ==================================================================================
class RobotGUI:
    def __init__(self, root):
        self.root = root; self.root.title("Chess Robot Monolith v8.20 (Fix Grip Crash)")
        self.root.geometry("600x850")
        self.robot = ChessRobotArm()
        self.mapper = BoardMapper(is_white_profile=True)
        self.setup_ui()
        self.b, self.s, self.e, self.w, self.g = 90, 90, 180, 18, GRIP_OPEN_ANGLE
        self.load_calib_ui()
        self.vis_running = False; self.vis_pts = {}; self.last_click = None

    def log(self, msg): self.txt.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {msg}\n"); self.txt.see(tk.END)

    def setup_ui(self):
        tabs = ttk.Notebook(self.root); tabs.pack(expand=1, fill="both")
        t_jog = ttk.Frame(tabs); tabs.add(t_jog, text='1. Jog & Calib')
        t_test = ttk.Frame(tabs); tabs.add(t_test, text='2. Test & Fix')
        t_game = ttk.Frame(tabs); tabs.add(t_game, text='3. Vision Game')

        # --- JOG & CALIB ---
        f_ctl = ttk.LabelFrame(t_jog, text="Manual Controls"); f_ctl.pack(pady=10)
        self.step = tk.IntVar(value=2)
        tk.Scale(f_ctl, from_=1, to=10, var=self.step, orient='h').pack()
        
        btns = [('W (Shld Up)',0,1,'w'), ('A (Base L)',1,0,'a'), ('S (Shld Dn)',1,1,'s'), ('D (Base R)',1,2,'d'),
                ('I (Elbow)',3,1,'i'), ('J (Extend)',4,0,'j'), ('K (Elbow)',4,1,'k'), ('L (Curl)',4,2,'l'),
                ('Open',5,0,'o'), ('Close',5,2,'p')]
        bf = ttk.Frame(f_ctl); bf.pack(pady=10)
        for t,r,c,k in btns: ttk.Button(bf, text=t, width=10, command=lambda k=k: self.jog(k)).grid(row=r, column=c)

        f_cal = ttk.LabelFrame(t_jog, text="Calibration Profile (Select to Edit)"); f_cal.pack(pady=10, fill='x')
        
        self.edit_profile_var = tk.BooleanVar(value=True) 
        row_prof = ttk.Frame(f_cal); row_prof.pack(pady=5)
        ttk.Radiobutton(row_prof, text="Edit WHITE Profile", variable=self.edit_profile_var, value=True, command=self.reload_mapper_ui).pack(side="left", padx=10)
        ttk.Radiobutton(row_prof, text="Edit BLACK Profile", variable=self.edit_profile_var, value=False, command=self.reload_mapper_ui).pack(side="left", padx=10)

        ttk.Button(f_cal, text="1. Home", command=self.move_home).pack(fill='x')
        for pt in ["a1", "h1", "h8", "a8"]: 
            ttk.Button(f_cal, text=f"Save {pt.upper()}", command=lambda p=pt: self.save_pt(p)).pack(fill='x')
        ttk.Button(f_cal, text="Save Calibration File", command=self.save_file).pack(fill='x')

        # --- TEST TAB ---
        f_tst = ttk.Frame(t_test); f_tst.pack(pady=10)
        self.ent_sq = ttk.Entry(f_tst); self.ent_sq.pack()
        ttk.Button(f_tst, text="Go To Square (Using Active Profile)", command=self.goto_sq).pack(pady=5)
        ttk.Button(f_tst, text="Override (Pin XYZ)", command=self.pin_sq).pack(fill='x', pady=2)
        ttk.Button(f_tst, text="Unpin (Delete Override)", command=self.unpin_sq).pack(fill='x')
        
        ttk.Label(f_tst, text="-- SIMULATOR --").pack(pady=10)
        self.ent_mv = ttk.Entry(f_tst); self.ent_mv.pack()
        self.chk_cap = tk.BooleanVar()
        ttk.Checkbutton(f_tst, text="Capture?", var=self.chk_cap).pack()
        ttk.Button(f_tst, text="Execute Move", command=self.run_sim).pack()

        # --- GAME TAB ---
        f_vis = ttk.LabelFrame(t_game, text="Vision"); f_vis.pack(pady=10, fill='x')
        ttk.Button(f_vis, text="Start Camera Stream", command=self.start_vision_stream).pack(fill='x')
        f_grid = ttk.Frame(f_vis); f_grid.pack()
        for i, pt in enumerate(['a8','h8','a1','h1']): 
            ttk.Button(f_grid, text=pt.upper(), command=lambda p=pt: self.assign_vis_pt(p)).grid(row=i//2, column=i%2)
        
        self.calib_side_var = tk.BooleanVar(value=True) 
        ttk.Radiobutton(f_vis, text="Vision: White Side", variable=self.calib_side_var, value=True).pack()
        ttk.Radiobutton(f_vis, text="Vision: Black Side", variable=self.calib_side_var, value=False).pack()
        ttk.Button(f_vis, text="Save & Close", command=self.stop_vision_stream).pack(fill='x')

        self.robot_color_var = tk.BooleanVar(value=True) 
        ttk.Label(t_game, text="Select Game Mode", font=("Arial", 10, "bold")).pack(pady=10)
        ttk.Radiobutton(t_game, text="Robot is WHITE (Moves First)", variable=self.robot_color_var, value=True).pack()
        ttk.Radiobutton(t_game, text="Robot is BLACK (Waits)", variable=self.robot_color_var, value=False).pack()
        
        ttk.Button(t_game, text="START GAME", command=self.start_game_thread).pack(fill='x', pady=10)
        
        btn_stop = tk.Button(t_game, text="STOP GAME & PARK", bg="#ffcccc", fg="red", font=("Arial", 10, "bold"), command=self.stop_game_gracefully)
        btn_stop.pack(fill='x', pady=5)

        self.txt = tk.Text(self.root, height=10); self.txt.pack(fill='both')

    def stop_game_gracefully(self):
        self.stop_requested = True
        self.log("STOP REQUESTED...")
        if hasattr(self, 'game_thread') and self.game_thread.is_alive():
            self.log("Game is running. Waiting for loop to finish and park...")
        else:
            self.log("Game not active. Parking immediately.")
            self.robot.park()

    def load_calib_ui(self):
        self.mapper = BoardMapper(is_white_profile=self.edit_profile_var.get())
        self.calib_pts = self.mapper.phys_cal.copy()
        p = "WHITE" if self.edit_profile_var.get() else "BLACK"
        self.log(f"Switched to {p} Profile.")

    def reload_mapper_ui(self):
        self.load_calib_ui()

    def save_pt(self, n): 
        x, y, z = get_xy_from_angles(self.b, self.s, self.e, self.w)
        self.calib_pts[n] = [x, y]
        self.log(f"Saved {n} [{x:.1f},{y:.1f}]")

    def save_file(self):
        is_white = self.edit_profile_var.get()
        fn = "calibration_white.json" if is_white else "calibration_black.json"
        json.dump(self.calib_pts, open(fn, 'w'))
        self.reload_mapper_ui()
        self.log(f"Saved to {fn}")

    def pin_sq(self):
        sq = self.ent_sq.get().strip().lower()
        x, y, z = get_xy_from_angles(self.b, self.s, self.e, self.w)
        d = self.mapper.ovr.copy()
        d[sq] = [x, y, z, self.b, self.s, self.e, self.w, "ANGLES"]
        
        fn = self.mapper.ovr_file
        json.dump(d, open(fn, 'w'))
        self.reload_mapper_ui()
        self.log(f"Pinned {sq} to {fn}")

    def unpin_sq(self):
        sq = self.ent_sq.get().strip().lower()
        d = self.mapper.ovr.copy()
        if sq in d: 
            del d[sq]
            json.dump(d, open(self.mapper.ovr_file, 'w'))
            self.reload_mapper_ui()
            self.log(f"Unpinned {sq}")

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
        if k=='o': self.g=GRIP_OPEN_ANGLE; 
        if k=='p': self.g=GRIP_CLOSED_ANGLE
        self.robot.send_cmd(self.b, self.s, self.e, self.w, self.g)

    def move_home(self):
        self.b, self.s, self.e, self.w, self.g = 90, 90, 180, 18, GRIP_OPEN_ANGLE
        self.robot.home()

    def goto_sq(self):
        sq = self.ent_sq.get().strip().lower()
        
        # 1. Setup Brain
        motion = RobotMotion(self.robot, self.mapper, lambda: False)
        
        # 2. Check Pinned
        if sq in self.mapper.ovr and len(self.mapper.ovr[sq]) > 5:
            val = self.mapper.ovr[sq]
            print(f"[GUI] Going to PINNED {sq}...")
            self.robot.send_cmd(val[3], val[4], val[5], val[6], GRIP_CLOSED_ANGLE)
            self.b, self.s, self.e, self.w = val[3], val[4], val[5], val[6]
            return

        # 3. Get Coordinates
        x, y, z_surface = self.mapper.get_coords(sq)
        print(f"[GUI] Attempting safe move to {sq}...")
        
        # --- STEP A: THE "SAFE VERTICAL" LIFT ---
        # FIX: Do not add +30. Instead, go to exactly 85 degrees.
        # 85 is high, upright, but keeps weight slightly forward to prevent toppling.
        SAFE_SHOULDER = 85 
        
        # Only move shoulder if we are currently LOWER than 85 (e.g. 40, 50, 60)
        # If we are already at 90 (Home), we just stay there (don't swing back).
        if self.s < SAFE_SHOULDER:
             self.robot.send_cmd(self.b, SAFE_SHOULDER, self.e, self.w, self.g)
             self.s = SAFE_SHOULDER
             time.sleep(0.05)

        # --- STEP B: ADAPTIVE TRAVEL ---
        # Try to find the highest reachable path
        travel_success = False
        
        # We try Z_HOVER (16), then 12, then 8.
        for try_z in [Z_HOVER, 12.0, 8.0]:
            print(f"   -> Trying travel at Height Z={try_z}...")
            angles = self.robot.inverse_kinematics(x, y, z_height=try_z)
            
            if angles:
                # We found a valid path
                self.robot.send_cmd(angles[0], angles[1], angles[2], angles[3], self.g)
                self.b, self.s, self.e, self.w = angles
                time.sleep(0.8)
                travel_success = True
                break 
        
        if not travel_success:
            print("[ERROR] CANNOT REACH SQUARE FROM ABOVE! Stopping.")
            return

        # --- STEP C: LAND ---
        motion.land_on_square(sq, self.g)
        
        # Update Internal State
        final_angles = self.robot.inverse_kinematics(x, y, z_height=z_surface)
        if final_angles: self.b, self.s, self.e, self.w = final_angles

    def start_vision_stream(self):
        if self.vis_running: return
        self.vis_running = True; self.vis_pts = {}; self.last_click = None
        self.calib_cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        cv2.namedWindow("Vision Setup")
        cv2.setMouseCallback("Vision Setup", self._mouse_cb)
        self._update_calibration_feed()

    def _mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN: self.last_click = [x, y]

    def _update_calibration_feed(self):
        if not self.vis_running: 
            if hasattr(self, 'calib_cap'): self.calib_cap.release()
            cv2.destroyWindow("Vision Setup")
            return
        ret, frame = self.calib_cap.read()
        if ret:
            if self.last_click: 
                cv2.drawMarker(frame, (self.last_click[0], self.last_click[1]), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
            for label, pt in self.vis_pts.items():
                x, y = int(pt[0]), int(pt[1])
                cv2.circle(frame, (x, y), 8, (0, 255, 0), -1) 
                cv2.putText(frame, label.upper(), (x+12, y-12), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4) 
                cv2.putText(frame, label.upper(), (x+12, y-12), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2) 
            cv2.imshow("Vision Setup", frame)
            cv2.waitKey(1)
        self.root.after(10, self._update_calibration_feed)

    def assign_vis_pt(self, sq):
        if self.last_click: 
            self.vis_pts[sq] = self.last_click
            self.log(f"Saved Vision Point: {sq.upper()}")
        else:
            self.log("Click on camera feed first!")

    def stop_vision_stream(self):
        self.vis_running = False
        self.root.after(100, self._finalize_vision_save)

    def _finalize_vision_save(self):
        try:
            self.v_zones = interpolate_grid(self.vis_pts['a8'], self.vis_pts['h8'], self.vis_pts['h1'], self.vis_pts['a1'])
            fn = "vision_white.json" if self.calib_side_var.get() else "vision_black.json"
            with open(fn, 'w') as f: json.dump(self.v_zones, f)
            with open("vision_grid.json", 'w') as f: json.dump(self.v_zones, f)
            self.log(f"Vision Saved to {fn}")
        except: 
            self.log("Vision Error: Missing Points?")
            messagebox.showerror("Error", "Please click all 4 corner buttons (A8, H8, A1, H1) before saving.")

    def start_game_thread(self):
        if hasattr(self, 'game_thread') and self.game_thread.is_alive(): return self.log("Game Running!")
        self.stop_requested = False
        self.game_thread = threading.Thread(target=run_vision_game, args=(self.robot, self.robot_color_var.get(), self))
        self.game_thread.start()

    def run_sim(self): threading.Thread(target=self._sim).start()
    
    # --- SIMULATOR: NOW JUST CALLS THE SHARED BRAIN ---
    def _sim(self):
        m = self.ent_mv.get().strip().lower(); cap = (self.chk_cap.get() == 1)
        
        # Instantiate the exact same brain as the game
        motion = RobotMotion(self.robot, self.mapper, lambda: False) # False means "No stop requested" for sim
        
        if len(m) < 4: return
        sq_src, sq_dst = m[0:2], m[2:4]
        
        if cap: motion.execute_capture(sq_src, sq_dst)
        else:   motion.execute_move(sq_src, sq_dst)
        
        self.robot.home()

if __name__ == "__main__":
    root = tk.Tk(); app = RobotGUI(root)
    def on_closing(): root.destroy(); os._exit(0)
    root.protocol("WM_DELETE_WINDOW", on_closing); root.mainloop()