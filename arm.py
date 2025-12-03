import math
import serial
import time

# --- ROBOT DIMENSIONS (cm) ---
L_BASE_H = 7.0     
L_SHOULDER = 17.7  
L_ELBOW = 19.5     # L3
L_WRIST = 17.0     # L4

# --- OFFSETS & LIMITS ---
OFFSET_BASE = 13.5   # Updated based on your 1550us center
OFFSET_SHOULDER = 4  
OFFSET_ELBOW = 0 
OFFSET_WRIST = 0 

# --- TILT CORRECTION ---
TILT_SLOPE = 0.25       
TILT_INTERCEPT = -5.25  
# NEW: Maximum amount we are allowed to drop the arm for near squares.
# Prevents the "Nose Dive" at D1/E1.
MAX_TILT_DROP = -2.5 

# Limits (Degrees)
# UPDATED: Increased Max to 145 to allow reaching D1/E1 easily
SHLD_MIN, SHLD_MAX = 0, 145  
ELBW_MIN, ELBW_MAX = 0, 180
WRST_MIN, WRST_MAX = 0, 180

# Torque Safety
MIN_SAFE_SHOULDER_ANGLE = 55.0

# --- SERVO CALIBRATION ---
SERVO_MIN_US = 400
SERVO_MAX_US = 2400
SERVO_RANGE_DEG = 180.0

class ChessRobotArm:
    def __init__(self, port='COM8'):
        self.connected = False
        try:
            self.ser = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)
            print(f"[HW] Connected to Robot on {port}")
            self.connected = True
        except Exception as e:
            print(f"[HW] Error connecting to {port}: {e}")

        self.gripper_state = SERVO_MIN_US 

    def deg_to_us(self, degrees):
        d = max(0, min(SERVO_RANGE_DEG, degrees))
        us = (d / SERVO_RANGE_DEG) * (SERVO_MAX_US - SERVO_MIN_US) + SERVO_MIN_US
        return int(us)

    def send_cmd(self, b, s, e, w, g):
        if not self.connected: return

        # 1. Apply Offsets & Clamp
        b_deg = max(0, min(180, b + OFFSET_BASE))
        s_deg = max(SHLD_MIN, min(SHLD_MAX, s + OFFSET_SHOULDER))
        e_deg = max(ELBW_MIN, min(ELBW_MAX, e + OFFSET_ELBOW))
        w_deg = max(WRST_MIN, min(WRST_MAX, w + OFFSET_WRIST))
        
        # 2. Convert to Microseconds
        b_us = self.deg_to_us(b_deg)
        s_us = self.deg_to_us(s_deg)
        e_us = self.deg_to_us(e_deg)
        w_us = self.deg_to_us(w_deg)
        g_us = self.deg_to_us(g) 

        msg = f"{b_us},{s_us},{e_us},{w_us},{g_us}\n"
        self.ser.write(msg.encode())
        
        while True:
            if self.ser.in_waiting:
                line = self.ser.readline().decode().strip()
                if line == "done": break

    def inverse_kinematics(self, x, y, z_height=5.0):
        base_angle = math.atan2(y, x) * 180 / math.pi
        r_total = math.sqrt(x**2 + y**2) 
        
        # --- SAFE TILT CORRECTION ---
        raw_correction = (r_total * TILT_SLOPE) + TILT_INTERCEPT
        
        # CLAMP: Don't let it drop lower than MAX_TILT_DROP (-2.5cm)
        # This ensures D1/E1 stay at least 2.5cm above the board (5.0 - 2.5)
        z_correction = max(MAX_TILT_DROP, raw_correction)
        
        z_final = z_height + z_correction
        
        # 2. Search Loop
        best_solution = None
        for pitch in range(-90, 5, 5): 
            result = self.try_reach(r_total, pitch, z_final)
            if result:
                s, e, w = result
                if s >= MIN_SAFE_SHOULDER_ANGLE:
                    return base_angle, s, e, w
                if best_solution is None or s > best_solution[1]:
                    best_solution = (base_angle, s, e, w)

        if best_solution:
            print(f"[IK] WARNING: Unsafe Shoulder={best_solution[1]:.1f}°")
            return best_solution

        print("[IK] Target Unreachable.")
        return None

    def try_reach(self, r_total, pitch_deg, z_target):
        rad_pitch = math.radians(pitch_deg)
        dr = L_WRIST * math.cos(rad_pitch)
        dz = L_WRIST * math.sin(rad_pitch)
        
        r_wrist = r_total - dr
        z_wrist = (z_target - dz) - L_BASE_H 
        
        h = math.sqrt(r_wrist**2 + z_wrist**2)
        if h > (L_SHOULDER + L_ELBOW): return None 

        try:
            alpha = math.acos((L_SHOULDER**2 + h**2 - L_ELBOW**2) / (2 * L_SHOULDER * h))
            beta = math.acos((L_SHOULDER**2 + L_ELBOW**2 - h**2) / (2 * L_SHOULDER * L_ELBOW))
        except ValueError: return None 

        theta_line = math.atan2(z_wrist, r_wrist)
        s = (theta_line + alpha) * 180 / math.pi
        e = beta * 180 / math.pi
        
        global_forearm = s - (180 - e)
        w = global_forearm - pitch_deg
        return s, e, w

    def move_to_xyz(self, x, y):
        angles = self.inverse_kinematics(x, y, z_height=5.0)
        if angles:
            b, s, e, w = angles
            self.send_cmd(b, s, e, w, self.gripper_state)
            return True
        return False
    
    def grasp(self): self.gripper_state = 180; pass
    def release(self): self.gripper_state = 0; pass
    def home(self): self.send_cmd(90, 90, 180, 0, 0)