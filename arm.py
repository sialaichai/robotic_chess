import math
import serial
import time

# ==================================================================================
# ROBOT ARM DRIVER (arm.py) - CALIBRATED FOR 1550/1400/2400
# ==================================================================================

# --- DIMENSIONS (cm) ---
L_BASE_H = 7.0     
L_SHOULDER = 17.7  
L_ELBOW = 19.5     
L_WRIST = 17.0     

# --- OFFSETS (CALIBRATED) ---
# Base 1550us = ~103.5 deg. IK Target is 90. Offset = 13.5
OFFSET_BASE = 13.5      
# Shoulder 1400us = 90 deg (Vertical). Matches math. Offset = 0
OFFSET_SHOULDER = 0.0  
# Elbow 2400us = 180 deg (Straight). Matches math. Offset = 0
OFFSET_ELBOW = 0.0
OFFSET_WRIST = 0.0

# --- HEIGHT STRATEGY ---
Z_TRAVEL = 8.0  
Z_GRIP = 5    

# --- TILT CORRECTION (DISABLED) ---
# We set these to 0.0 so arm.py acts as a "slave" to the Monolith.
TILT_SLOPE = 0.0       
TILT_INTERCEPT = 0.0  
MAX_TILT_DROP = -50.0  # Set low enough so it never limits the movement

# --- LIMITS ---
SHLD_MIN, SHLD_MAX = 0, 145  
ELBW_MIN, ELBW_MAX = 0, 180
WRST_MIN, WRST_MAX = 0, 180
MIN_SAFE_SHOULDER_ANGLE = 55.0

# --- SERVO PULSE MAP ---
SERVO_MIN_US = 400
SERVO_MAX_US = 2400
SERVO_RANGE_DEG = 180.0

class ChessRobotArm:
    def __init__(self):
        # ... (keep your existing serial connection code) ...
        try:
            self.ser = serial.Serial('COM3', 9600, timeout=1)
            time.sleep(2) # Wait for Arduino to reset
        except:
            print("Robot not connected")

        # --- NEW CODE: DEFINE STARTING POSITION ---
        # This matches the "HOME" values in your Arduino code.
        # Now the robot knows where to interpolate FROM.
        self.cur_b = 1550.0 
        self.cur_s = 1400.0
        self.cur_e = 2400.0
        self.cur_w = 600.0 
        self.gripper_state = 180 # Open
        
    def deg_to_us(self, degrees):
        """Converts degrees to microseconds based on constants."""
        d = max(0, min(SERVO_RANGE_DEG, degrees))
        return int((d / SERVO_RANGE_DEG) * (SERVO_MAX_US - SERVO_MIN_US) + SERVO_MIN_US)

    def send_cmd(self, b, s, e, w, g):
        if not self.connected: return
        
        # Apply Hardware Offsets
        b_final = b + OFFSET_BASE
        s_final = s + OFFSET_SHOULDER
        e_final = e + OFFSET_ELBOW
        w_final = w + OFFSET_WRIST
        
        # Clamp Logic
        b_deg = max(0, min(180, b_final))
        s_deg = max(SHLD_MIN, min(SHLD_MAX, s_final))
        e_deg = max(ELBW_MIN, min(ELBW_MAX, e_final))
        w_deg = max(WRST_MIN, min(WRST_MAX, w_final))
        
        # Send formatted string
        msg = f"{self.deg_to_us(b_deg)},{self.deg_to_us(s_deg)},{self.deg_to_us(e_deg)},{self.deg_to_us(w_deg)},{self.deg_to_us(g)}\n"
        self.ser.write(msg.encode())
        self._wait_for_done()

    def _wait_for_done(self):
        """Blocks until Arduino sends 'done'."""
        if not self.connected: return
        while True:
            if self.ser.in_waiting:
                line = self.ser.readline().decode().strip()
                if line == "done": 
                    break

    def park(self):
        """Moves robot to the calibrated parking position."""
        if not self.connected: return
        print("[HW] Parking Robot...")
        
        # PARKING POSE (Deg)
        # Base: 90, Shoulder: 70, Elbow: 80, Wrist: 80, Gripper: 34
        # Note: We must include offsets manually here if we use raw calculation, 
        # or just use send_cmd which now handles offsets correctly.
        # Since 90 base + 13.5 offset = 1550 (Forward), this is safe.
        
        # We use send_cmd to utilize the new offsets automatically
        self.send_cmd(90, 70, 80, 80, 34)

    def inverse_kinematics(self, x, y, z_height=Z_TRAVEL):
        # FIX: Add 90 degrees so that x-axis (Forward) maps to 90 (Center)
        base_angle = (math.atan2(y, x) * 180 / math.pi) + 90
        
        r_total = math.sqrt(x**2 + y**2) 
        
        raw_correction = (r_total * TILT_SLOPE) + TILT_INTERCEPT
        z_final = z_height + max(MAX_TILT_DROP, raw_correction)
        
        best_solution = None
        for pitch in range(-90, 5, 5): 
            rad_pitch = math.radians(pitch)
            dr = L_WRIST * math.cos(rad_pitch)
            dz = L_WRIST * math.sin(rad_pitch)
            r_wrist = r_total - dr
            z_wrist = (z_final - dz) - L_BASE_H 
            
            h = math.sqrt(r_wrist**2 + z_wrist**2)
            if h > (L_SHOULDER + L_ELBOW): continue

            try:
                alpha = math.acos((L_SHOULDER**2 + h**2 - L_ELBOW**2) / (2 * L_SHOULDER * h))
                beta = math.acos((L_SHOULDER**2 + L_ELBOW**2 - h**2) / (2 * L_SHOULDER * L_ELBOW))
            except ValueError: continue

            theta_line = math.atan2(z_wrist, r_wrist)
            s = (theta_line + alpha) * 180 / math.pi
            e = beta * 180 / math.pi
            w = (s - (180 - e)) - pitch 
            
            if s >= MIN_SAFE_SHOULDER_ANGLE: return base_angle, s, e, w
            if best_solution is None or s > best_solution[1]: best_solution = (base_angle, s, e, w)

        return best_solution

    def move_to_xyz(self, x, y, z=Z_TRAVEL, steps=100):
        """
        Moves to coordinates with SMOOTH interpolation.
        """
        target_angles = self.inverse_kinematics(x, y, z_height=z)
        if not target_angles: 
            print(f"DEBUG: No IK Solution for {x:.1f}, {y:.1f}, {z:.1f}")
            return

        # 1. SETUP STARTING ANGLES
        # If (rarely) we still don't have a position, use target (snap)
        if not hasattr(self, 'cur_b'): 
            self.cur_b, self.cur_s, self.cur_e, self.cur_w = target_angles

        # 2. CALCULATE INCREMENTS
        b_diff = (target_angles[0] - self.cur_b) / steps
        s_diff = (target_angles[1] - self.cur_s) / steps
        e_diff = (target_angles[2] - self.cur_e) / steps
        w_diff = (target_angles[3] - self.cur_w) / steps
        
        g = getattr(self, 'gripper_state', 180)

        # 3. EXECUTE SMOOTH LOOP
        for i in range(steps):
            self.cur_b += b_diff
            self.cur_s += s_diff
            self.cur_e += e_diff
            self.cur_w += w_diff
            
            self.send_cmd(self.cur_b, self.cur_s, self.cur_e, self.cur_w, g)
            
            # CRITICAL: 9600 baud takes ~25ms per command. 
            # We sleep 40ms to keep the buffer clean.
            time.sleep(0.04) 

        # 4. FINAL LOCK
        self.cur_b, self.cur_s, self.cur_e, self.cur_w = target_angles
        self.send_cmd(self.cur_b, self.cur_s, self.cur_e, self.cur_w, g)
    def grasp(self): 
        self.gripper_state = self.deg_to_us(180) # Closed
    
    def release(self): 
        self.gripper_state = self.deg_to_us(34)  # Open
    
    def home(self): 
        # Base 90 (1550us), Shoulder 90 (1400us), Elbow 180 (2400us)
        # We send geometric 90/90/180. The offsets handle the conversion.
        self.send_cmd(90, 90, 180, 18, 34)