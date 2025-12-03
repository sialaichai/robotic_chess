import tkinter as tk
from tkinter import ttk
import json
import os
import time
import threading
import math
from arm import ChessRobotArm
import main as master_script 

# --- CONFIG ---
ROBOT_CALIB = "calibration.json"
OVERRIDES_FILE = "overrides.json"

# --- GLOBAL HELPER (Same as main.py) ---
def get_xy_from_angles(b, s, e, w):
    # Must match arm.py dimensions exactly
    L1 = 7.0; L2 = 17.7; L3 = 19.5; L4 = 17.0
    
    rad_b = math.radians(b)
    rad_s = math.radians(s)
    global_e = s - (180 - e)
    rad_e = math.radians(global_e)
    global_w = global_e - w
    rad_w = math.radians(global_w)

    r = (L2 * math.cos(rad_s)) + (L3 * math.cos(rad_e)) + (L4 * math.cos(rad_w))
    
    x = r * math.cos(rad_b)
    y = r * math.sin(rad_b)
    return x, y

class BoardMapper:
    def __init__(self, cal_data):
        self.a1 = cal_data['a1']
        self.h8 = cal_data['h8']
        self.x_step = (self.h8[0] - self.a1[0]) / 7.0
        self.y_step = (self.h8[1] - self.a1[1]) / 7.0
        
        self.overrides = {}
        if os.path.exists(OVERRIDES_FILE):
            with open(OVERRIDES_FILE, 'r') as f:
                self.overrides = json.load(f)
        
    def get_coords(self, square_name):
        if square_name in self.overrides:
            return self.overrides[square_name]
        col = ord(square_name[0]) - ord('a')
        row = int(square_name[1]) - 1
        x = self.a1[0] + (col * self.x_step)
        y = self.a1[1] + (row * self.y_step)
        return x, y

class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Chess Robot Control Panel")
        self.root.geometry("600x850")
        
        # 1. SETUP UI
        self.setup_ui()

        # 2. STATE
        self.b = 90; self.s = 90; self.e = 180; self.w = 0; self.g = 0
        self.calib_corners = []
        self.last_target_xy = None

        # 3. CONNECT
        try:
            self.robot = ChessRobotArm(port='COM8')
            if self.robot.connected: self.log("Robot Connected (COM8)")
            else: self.log("WARNING: Robot Serial Failed")
        except Exception as e:
            self.robot = None
            self.log(f"Connection Error: {e}")

    def log(self, msg):
        ts = time.strftime("%H:%M:%S")
        self.txt_log.insert(tk.END, f"[{ts}] {msg}\n")
        self.txt_log.see(tk.END)
        print(msg)

    def setup_ui(self):
        tabs = ttk.Notebook(self.root)
        self.tab_jog = ttk.Frame(tabs)
        self.tab_test = ttk.Frame(tabs)
        self.tab_game = ttk.Frame(tabs)
        
        tabs.add(self.tab_jog, text='1. Jog & Calibrate')
        tabs.add(self.tab_test, text='2. Test & Fix')
        tabs.add(self.tab_game, text='3. Vision & Game')
        tabs.pack(expand=1, fill="both")
        
        # --- TAB 1: JOG ---
        frame_jog = ttk.LabelFrame(self.tab_jog, text="Manual Control")
        frame_jog.pack(pady=10, padx=10, fill="x")
        
        self.step_var = tk.IntVar(value=2)
        f_step = ttk.Frame(frame_jog); f_step.pack(pady=5)
        ttk.Label(f_step, text="Step (deg):").pack(side="left")
        tk.Scale(f_step, from_=1, to=10, orient="horizontal", variable=self.step_var).pack(side="left")

        btn_frame = ttk.Frame(frame_jog); btn_frame.pack(pady=10)
        
        # Grid Layout
        ttk.Button(btn_frame, text="Shld UP (W)", width=12, command=lambda: self.jog('w')).grid(row=0, column=1)
        ttk.Button(btn_frame, text="Base LEFT (A)", width=12, command=lambda: self.jog('a')).grid(row=1, column=0)
        ttk.Button(btn_frame, text="Shld DOWN (S)", width=12, command=lambda: self.jog('s')).grid(row=1, column=1)
        ttk.Button(btn_frame, text="Base RIGHT (D)", width=12, command=lambda: self.jog('d')).grid(row=1, column=2)
        
        ttk.Label(btn_frame, text="---").grid(row=2, column=1)
        
        ttk.Button(btn_frame, text="Elbow EXT (I)", width=12, command=lambda: self.jog('i')).grid(row=3, column=1)
        ttk.Button(btn_frame, text="Wrist DOWN (J)", width=12, command=lambda: self.jog('j')).grid(row=4, column=0)
        ttk.Button(btn_frame, text="Elbow BEND (K)", width=12, command=lambda: self.jog('k')).grid(row=4, column=1)
        ttk.Button(btn_frame, text="Wrist UP (L)", width=12, command=lambda: self.jog('l')).grid(row=4, column=2)
        
        ttk.Button(btn_frame, text="OPEN (O)", width=12, command=lambda: self.jog('o')).grid(row=5, column=0)
        ttk.Button(btn_frame, text="CLOSE (P)", width=12, command=lambda: self.jog('p')).grid(row=5, column=2)

        f_cal = ttk.LabelFrame(self.tab_jog, text="Calibration")
        f_cal.pack(pady=10, padx=10, fill="x")
        ttk.Button(f_cal, text="[1] Reset", command=self.calib_reset).pack(fill="x", pady=2)
        ttk.Button(f_cal, text="[2] Save A1", command=lambda: self.calib_save("A1")).pack(fill="x", pady=2)
        ttk.Button(f_cal, text="[3] Save H8", command=lambda: self.calib_save("H8")).pack(fill="x", pady=2)
        ttk.Button(f_cal, text="[4] Save File", command=self.calib_finish).pack(fill="x", pady=2)

        # --- TAB 2: TEST & FIX ---
        f_test = ttk.Frame(self.tab_test); f_test.pack(pady=10, padx=20)
        
        # Section A: Single Square
        ttk.Label(f_test, text="-- SINGLE SQUARE TEST --", font=("Arial", 10, "bold")).pack(pady=5)
        ttk.Label(f_test, text="Target Square:").pack()
        self.ent_target = ttk.Entry(f_test, font=("Arial", 12)); self.ent_target.pack(pady=5)
        ttk.Button(f_test, text="GO TO SQUARE", command=self.test_goto).pack(pady=5, fill="x")
        
        # Section B: Correction
        ttk.Label(f_test, text="-- CORRECTION TOOLS --", font=("Arial", 10, "bold")).pack(pady=10)
        ttk.Button(f_test, text="FIX GRID (Shift Whole Board)", command=self.test_fix).pack(pady=2, fill="x")
        ttk.Button(f_test, text="OVERRIDE SQUARE (Pin Point)", command=self.save_override).pack(pady=2, fill="x")
        ttk.Button(f_test, text="RESET SQUARE (Remove Pin)", command=self.del_override).pack(pady=2, fill="x")

        # Section C: Move Simulator
        ttk.Separator(f_test).pack(fill='x', pady=15)
        ttk.Label(f_test, text="-- MOVE SIMULATOR --", font=("Arial", 10, "bold")).pack(pady=5)
        
        f_sim = ttk.Frame(f_test); f_sim.pack(fill="x")
        ttk.Label(f_sim, text="Move (e.g. e2e4):").pack(side="left")
        self.ent_move = ttk.Entry(f_sim, width=10); self.ent_move.pack(side="left", padx=5)
        
        self.chk_capture_var = tk.BooleanVar()
        ttk.Checkbutton(f_sim, text="Capture?", variable=self.chk_capture_var).pack(side="left", padx=5)
        
        ttk.Button(f_test, text="EXECUTE TEST MOVE", command=self.run_move_test).pack(pady=5, fill="x")

        # --- TAB 3: GAME ---
        f_game = ttk.Frame(self.tab_game); f_game.pack(pady=20, padx=20)
        ttk.Button(f_game, text="Calibrate Camera", command=self.run_cam).pack(pady=10, fill="x")
        ttk.Button(f_game, text="START GAME", command=self.run_game).pack(pady=10, fill="x")
        
        self.txt_log = tk.Text(self.root, height=12, bg="#f0f0f0", font=("Consolas", 9))
        self.txt_log.pack(padx=10, pady=5, fill="both", expand=True)

    # --- LOGIC ---
    def jog(self, k):
        step = self.step_var.get()
        if k=='w': self.s+=step
        if k=='s': self.s-=step
        if k=='a': self.b+=step
        if k=='d': self.b-=step
        if k=='i': self.e+=step
        if k=='k': self.e-=step
        if k=='j': self.w-=step
        if k=='l': self.w+=step
        if k=='o': self.g=0
        if k=='p': self.g=180
        
        self.s = max(0, min(145, self.s))
        self.e = max(0, min(180, self.e))
        self.w = max(0, min(180, self.w))
        if self.robot: self.robot.send_cmd(self.b, self.s, self.e, self.w, self.g)
        self.log(f"Jog: {self.b} {self.s} {self.e} {self.w}")

    def calib_reset(self):
        self.calib_corners = []
        self.b=90; self.s=90; self.e=180; self.w=0
        if self.robot: self.robot.home()
        self.log("Reset Done.")

    def calib_save(self, name):
        x, y = get_xy_from_angles(self.b, self.s, self.e, self.w)
        self.calib_corners.append((x,y))
        self.log(f"Captured {name}: {x:.1f}, {y:.1f}")

    def calib_finish(self):
        if len(self.calib_corners)<2: return self.log("Need 2 pts.")
        data = {"a1": self.calib_corners[-2], "h8": self.calib_corners[-1]}
        with open(ROBOT_CALIB, "w") as f: json.dump(data, f)
        self.log("Calibration Saved.")

    def test_goto(self):
        sq = self.ent_target.get().lower()
        if not os.path.exists(ROBOT_CALIB): return self.log("No Calib File.")
        with open(ROBOT_CALIB,'r') as f: mapper = BoardMapper(json.load(f))
        try:
            tx, ty = mapper.get_coords(sq)
            self.last_target_xy = (tx, ty)
            self.log(f"Going to {sq}...")
            self.robot.gripper_state = 180
            self.robot.home()
            time.sleep(0.5)
            self.robot.move_to_xyz(tx, ty)
            sol = self.robot.inverse_kinematics(tx, ty)
            if sol: self.b, self.s, self.e, self.w = sol; self.g=180
        except Exception as e: self.log(str(e))

    def test_fix(self):
        if not self.last_target_xy: return self.log("No previous move.")
        rx, ry = get_xy_from_angles(self.b, self.s, self.e, self.w)
        tx, ty = self.last_target_xy
        dx, dy = rx-tx, ry-ty
        with open(ROBOT_CALIB,'r') as f: data = json.load(f)
        data['a1'] = (data['a1'][0]+dx, data['a1'][1]+dy)
        data['h8'] = (data['h8'][0]+dx, data['h8'][1]+dy)
        with open(ROBOT_CALIB,'w') as f: json.dump(data, f)
        self.log(f"Grid Shifted: X{dx:.2f} Y{dy:.2f}")

    def save_override(self):
        sq = self.ent_target.get().lower()
        if not sq: return self.log("Enter Square Name.")
        x, y = get_xy_from_angles(self.b, self.s, self.e, self.w)
        ovr = {}
        if os.path.exists(OVERRIDES_FILE):
            with open(OVERRIDES_FILE,'r') as f: ovr = json.load(f)
        ovr[sq] = (x, y)
        with open(OVERRIDES_FILE,'w') as f: json.dump(ovr, f)
        self.log(f"Pinned {sq} at {x:.1f},{y:.1f}")

    def del_override(self):
        sq = self.ent_target.get().lower()
        if not os.path.exists(OVERRIDES_FILE): return
        with open(OVERRIDES_FILE,'r') as f: ovr = json.load(f)
        if sq in ovr:
            del ovr[sq]
            with open(OVERRIDES_FILE,'w') as f: json.dump(ovr, f)
            self.log(f"Unpinned {sq}")

    # --- MOVE SIMULATOR ---
    def run_move_test(self):
        move_str = self.ent_move.get().strip().lower()
        is_capture = self.chk_capture_var.get()
        
        if len(move_str) != 4:
            return self.log("Error: Format must be 4 chars (e.g. e2e4)")
            
        sq_start = move_str[0:2]
        sq_end = move_str[2:4]
        
        self.log(f"Simulating: {sq_start} -> {sq_end} (Cap: {is_capture})")
        threading.Thread(target=self._sim_thread, args=(sq_start, sq_end, is_capture)).start()

    def _sim_thread(self, sq1, sq2, is_capture):
        if not os.path.exists(ROBOT_CALIB): return self.log("No Calib.")
        with open(ROBOT_CALIB,'r') as f: mapper = BoardMapper(json.load(f))
        
        try:
            x1, y1 = mapper.get_coords(sq1)
            x2, y2 = mapper.get_coords(sq2)
            
            # CAPTURE LOGIC
            if is_capture:
                self.log("Capturing Enemy...")
                self.robot.release()
                self.robot.move_to_xyz(x2, y2) # Go to enemy
                time.sleep(0.5); self.robot.grasp()
                
                # Dump Bin location (Left side)
                self.robot.send_cmd(180, 90, 90, 90, 180) 
                time.sleep(1.0); self.robot.release()
                time.sleep(0.5)

            # MOVE LOGIC
            self.log(f"Moving {sq1} to {sq2}...")
            self.robot.release()
            self.robot.move_to_xyz(x1, y1) # Go to Start
            time.sleep(1.0); self.robot.grasp()
            self.robot.move_to_xyz(x1, y1)
            time.sleep(0.5)

            # LIFT
            self.robot.send_cmd(90, 110, 160, 45, 180)
            time.sleep(0.5)

            # PLACE
            self.robot.move_to_xyz(x2, y2)
            time.sleep(1.0)
            self.robot.release()
            self.robot.move_to_xyz(x2, y2)
            
            time.sleep(0.5)
            self.robot.home()
            self.log("Move Complete.")

        except Exception as e:
            self.log(f"Sim Error: {e}")

    # --- THREADS ---
    def run_cam(self): threading.Thread(target=self._cam, daemon=True).start()
    def _cam(self): master_script.mode_cam_calib()
    
    def run_game(self): threading.Thread(target=self._game, daemon=True).start()
    def _game(self): master_script.mode_play_game(self.robot)

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotGUI(root)
    root.mainloop()