import tkinter as tk
from tkinter import ttk
import json
import os
import time
import threading
import math
from arm import ChessRobotArm, Z_TRAVEL, Z_GRIP
import main as master_script 

# --- CONFIG ---
ROBOT_CALIB = "calibration.json"
OVERRIDES_FILE = "overrides.json"

# --- HELPER: Forward Kinematics (SYNCED WITH ARM.PY) ---
def get_xy_from_angles(b, s, e, w):
    L1 = 7.0; L2 = 17.7; L3 = 19.5; L4 = 17.0
    
    # Base Offset REMOVED to match arm.py (Was 13.5, now 0)
    # Shoulder Offset KEPT to match arm.py (Gravity compensation)
    OFFSET_BASE = 0.0
    OFFSET_SHOULDER = 4.0
    
    true_b = b + OFFSET_BASE
    true_s = s + OFFSET_SHOULDER

    rb, rs = math.radians(true_b), math.radians(true_s)
    ge = true_s - (180 - e); re = math.radians(ge)
    gw = ge - w; rw = math.radians(gw)
    r = (L2*math.cos(rs))+(L3*math.cos(re))+(L4*math.cos(rw))
    return r*math.cos(rb), r*math.sin(rb)

class BoardMapper:
    def __init__(self, cal_data):
        self.a1 = cal_data['a1']; self.h8 = cal_data['h8']
        self.x_step = (self.h8[0] - self.a1[0]) / 7.0
        self.y_step = (self.h8[1] - self.a1[1]) / 7.0
        self.ovr = {}
        if os.path.exists(OVERRIDES_FILE):
            with open(OVERRIDES_FILE,'r') as f: self.ovr = json.load(f)
    def get_coords(self, sq):
        if sq in self.ovr: return self.ovr[sq]
        c = ord(sq[0])-ord('a'); r = int(sq[1])-1
        return self.a1[0]+(c*self.x_step), self.a1[1]+(r*self.y_step)

class RobotGUI:
    def __init__(self, root):
        self.root = root; self.root.geometry("600x850")
        self.root.title("Chess Robot Control Panel v4.1 (Clean)")
        self.setup_ui()
        
        self.b=90; self.s=90; self.e=180; self.w=18; self.g=34 
        self.calib_points = {} # Dict for safe storage
        self.last_target_xy = None
        
        try:
            self.robot = ChessRobotArm(port='COM8')
            if self.robot.connected: self.log("Robot Connected")
        except Exception as e:
            self.robot = None; self.log(str(e))

    def log(self, msg):
        self.txt_log.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {msg}\n")
        self.txt_log.see(tk.END)

    def setup_ui(self):
        tabs = ttk.Notebook(self.root)
        self.t_jog = ttk.Frame(tabs); tabs.add(self.t_jog, text='1. Jog')
        self.t_test = ttk.Frame(tabs); tabs.add(self.t_test, text='2. Test')
        self.t_game = ttk.Frame(tabs); tabs.add(self.t_game, text='3. Game')
        tabs.pack(expand=1, fill="both")

        # JOG
        f_jog = ttk.LabelFrame(self.t_jog, text="Manual Control"); f_jog.pack(pady=10)
        self.step = tk.IntVar(value=2)
        tk.Scale(f_jog, from_=1, to=10, orient="horizontal", variable=self.step).pack()
        
        btns = [('W (Shld Up)',0,1,'w'), ('A (Base L)',1,0,'a'), ('S (Shld Dn)',1,1,'s'), ('D (Base R)',1,2,'d'),
                ('I (Elbow)',3,1,'i'), ('J (EXTEND)',4,0,'j'), ('K (Elbow)',4,1,'k'), ('L (CURL)',4,2,'l'),
                ('Open',5,0,'o'), ('Close',5,2,'p')]
        bf = ttk.Frame(f_jog); bf.pack(pady=10)
        for t,r,c,k in btns: ttk.Button(bf, text=t, width=10, command=lambda k=k: self.jog(k)).grid(row=r, column=c, padx=2, pady=2)

        f_cal = ttk.LabelFrame(self.t_jog, text="Calibration Actions"); f_cal.pack(pady=10, fill="x", padx=10)
        ttk.Button(f_cal, text="[1] Reset", command=self.calib_reset).pack(fill="x")
        ttk.Button(f_cal, text="[2] Save Position as A1", command=lambda: self.calib_save("a1")).pack(fill="x")
        ttk.Button(f_cal, text="[3] Save Position as H8", command=lambda: self.calib_save("h8")).pack(fill="x")
        ttk.Button(f_cal, text="[4] Save File", command=self.calib_finish).pack(fill="x")

        # TEST
        f_test = ttk.Frame(self.t_test); f_test.pack(pady=10)
        self.ent_target = ttk.Entry(f_test); self.ent_target.pack()
        ttk.Button(f_test, text="Go To Square", command=self.test_goto).pack(pady=5)
        ttk.Button(f_test, text="Fix Grid (Shift)", command=self.test_fix).pack(fill="x")
        ttk.Button(f_test, text="Override (Pin)", command=self.save_override).pack(fill="x")
        ttk.Button(f_test, text="Reset Pin", command=self.del_override).pack(fill="x")
        
        ttk.Label(f_test, text="-- SIMULATOR --", font="bold").pack(pady=10)
        self.ent_move = ttk.Entry(f_test); self.ent_move.pack()
        self.chk_cap = tk.BooleanVar()
        ttk.Checkbutton(f_test, text="Capture?", variable=self.chk_cap).pack()
        ttk.Button(f_test, text="Execute Move", command=self.run_move_test).pack(pady=5)

        # GAME
        f_gm = ttk.Frame(self.t_game); f_gm.pack(pady=10)
        ttk.Button(f_gm, text="Calibrate Camera", command=lambda: threading.Thread(target=master_script.mode_cam_calib).start()).pack(pady=5)
        ttk.Button(f_gm, text="Start Game Loop", command=lambda: threading.Thread(target=master_script.mode_play_game, args=(self.robot,)).start()).pack(pady=5)
        self.txt_log = tk.Text(self.root, height=10); self.txt_log.pack(fill="both", padx=5, pady=5)

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
        self.s = max(0, min(145, self.s)); self.e = max(0, min(180, self.e)); self.w = max(0, min(180, self.w))
        if self.robot: self.robot.send_cmd(self.b, self.s, self.e, self.w, self.g)
        self.log(f"J: {self.b} {self.s} {self.e} {self.w}")

    def calib_reset(self): 
        self.calib_points={}; self.b=90; self.s=90; self.e=180; self.w=18; self.g=34
        if self.robot: self.robot.home()
        self.log("Reset.")

    def calib_save(self, name): 
        self.calib_points[name] = get_xy_from_angles(self.b,self.s,self.e,self.w)
        self.log(f"Saved {name.upper()}")

    def calib_finish(self): 
        if "a1" not in self.calib_points or "h8" not in self.calib_points: return self.log("Error: Need A1 & H8.")
        json.dump({"a1":self.calib_points["a1"],"h8":self.calib_points["h8"]}, open(ROBOT_CALIB,'w'))
        self.log("Saved Calib.")
    
    def test_goto(self):
        sq = self.ent_target.get()
        if not os.path.exists(ROBOT_CALIB): return self.log("No Calib.")
        mapper = BoardMapper(json.load(open(ROBOT_CALIB)))
        x, y = mapper.get_coords(sq)
        self.last_target_xy = (x,y)
        self.robot.gripper_state = 180
        self.robot.move_to_xyz(x, y, z=Z_TRAVEL); time.sleep(0.5); self.robot.move_to_xyz(x, y, z=Z_GRIP)
        angles = self.robot.inverse_kinematics(x, y, z_height=Z_GRIP)
        if angles: self.b, self.s, self.e, self.w = angles; self.g = 180; self.log(f"Arrived {sq}. Synced.")
    
    def test_fix(self):
        if not self.last_target_xy: return
        rx, ry = get_xy_from_angles(self.b, self.s, self.e, self.w)
        dx, dy = rx-self.last_target_xy[0], ry-self.last_target_xy[1]
        d = json.load(open(ROBOT_CALIB))
        d['a1'] = [d['a1'][0]+dx, d['a1'][1]+dy]; d['h8'] = [d['h8'][0]+dx, d['h8'][1]+dy]
        json.dump(d, open(ROBOT_CALIB,'w')); self.log("Fixed.")

    def save_override(self):
        sq = self.ent_target.get(); d = {}
        if os.path.exists(OVERRIDES_FILE): d = json.load(open(OVERRIDES_FILE))
        d[sq] = get_xy_from_angles(self.b, self.s, self.e, self.w)
        json.dump(d, open(OVERRIDES_FILE,'w')); self.log(f"Pinned {sq}")

    def del_override(self):
        sq = self.ent_target.get(); d = {}
        if os.path.exists(OVERRIDES_FILE): 
            d = json.load(open(OVERRIDES_FILE))
            if sq in d: del d[sq]; json.dump(d, open(OVERRIDES_FILE,'w')); self.log(f"Unpinned {sq}")

    def run_move_test(self):
        threading.Thread(target=self._sim, args=(self.ent_move.get(), self.chk_cap.get())).start()

    def _sim(self, m, cap):
        if not os.path.exists(ROBOT_CALIB): return self.log("No Calib.")
        mapper = BoardMapper(json.load(open(ROBOT_CALIB)))
        x1, y1 = mapper.get_coords(m[0:2]); x2, y2 = mapper.get_coords(m[2:4])
        if cap:
            self.log("Capturing..."); self.robot.release(); self.robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(1)
            self.robot.move_to_xyz(x2, y2, z=Z_GRIP); time.sleep(0.5); self.robot.grasp(); time.sleep(0.5)
            self.robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(0.5)
            self.robot.send_cmd(180, 90, 90, 90, 180); time.sleep(1); self.robot.release(); time.sleep(0.5)
        
        self.log("Moving..."); self.robot.release(); self.robot.move_to_xyz(x1, y1, z=Z_TRAVEL); time.sleep(1)
        self.robot.move_to_xyz(x1, y1, z=Z_GRIP); time.sleep(0.5); self.robot.grasp(); time.sleep(0.5)
        self.robot.move_to_xyz(x1, y1, z=Z_TRAVEL); time.sleep(0.5); self.robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(1)
        self.robot.move_to_xyz(x2, y2, z=Z_GRIP); time.sleep(0.5); self.robot.release(); time.sleep(0.5)
        self.robot.move_to_xyz(x2, y2, z=Z_TRAVEL); time.sleep(0.5); self.robot.home(); self.log("Done.")

if __name__ == "__main__":
    root = tk.Tk(); app = RobotGUI(root); root.mainloop()