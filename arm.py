import math
import serial
import time

# ==================================================================================
# ROBOT ARM DRIVER (arm.py)
# ----------------------------------------------------------------------------------
# This class handles low-level communication with the Arduino.
# It converts "Chess Squares" (X, Y coordinates) into "Servo Angles" (Inverse Kinematics).
# It also enforces safety limits to prevent the robot from hitting itself or burning motors.
# ==================================================================================

# --- PHYSICAL DIMENSIONS (cm) ---
# accurate measurements are critical for the math to work!
L_BASE_H = 7.0     # Height from table to shoulder pivot
L_SHOULDER = 17.7  # Length of upper arm
L_ELBOW = 19.5     # Length of forearm (L3)
L_WRIST = 17.0     # Length from wrist pivot to gripper tip (L4)

# --- HARDWARE OFFSETS ---
# If the robot moves slightly left/right/up/down consistently, tweak these.
OFFSET_BASE = 13.5   # Corrects the base rotation center (1550us alignment)
OFFSET_SHOULDER = 4  # Lifts the arm slightly to counteract gravity sag
OFFSET_ELBOW = 0 
OFFSET_WRIST = 0 

# --- TILT CORRECTION (Advanced Gravity Compensation) ---
# Physical arms "sag" when extended due to weight/gravity.
# This formula tilts the movement plane to match reality.
# Slope 0.25: Lifts the arm higher the further out it reaches.
# Intercept -5.25: Lowers the arm for close squares to prevent hovering too high.
TILT_SLOPE = 0.25       
TILT_INTERCEPT = -5.25  
MAX_TILT_DROP = -2.5 # Safety floor: Never drop the arm lower than this for near squares.

# --- SAFETY LIMITS (Degrees) ---
# These prevent the servos from hitting physical stops or the table.
SHLD_MIN, SHLD_MAX = 0, 145  
ELBW_MIN, ELBW_MAX = 0, 180
WRST_MIN, WRST_MAX = 0, 180

# --- TORQUE PROTECTION ---
# CRITICAL: Prevents the shoulder from leaning too far forward (Torque Spike).
# If the math requests a shoulder angle < 55 degrees (leaning way forward),
# the code will reject it and try to flatten the wrist instead.
MIN_SAFE_SHOULDER_ANGLE = 55.0

# --- SERVO CALIBRATION (Microseconds) ---
# Mapping 0-180 degrees to the specific pulse widths of your MG996R servos.
# We use 400-2400us based on your specific calibration data.
SERVO_MIN_US = 400
SERVO_MAX_US = 2400
SERVO_RANGE_DEG = 180.0

class ChessRobotArm:
    def __init__(self, port='COM8'):
        """Initializes the serial connection to Arduino."""
        self.connected = False
        try:
            self.ser = serial.Serial(port, 9600, timeout=1)
            time.sleep(2) # Wait for Arduino to reboot
            print(f"[HW] Connected to Robot on {port}")
            self.connected = True
        except Exception as e:
            print(f"[HW] Error connecting to {port}: {e}")
            print("[HW] Running in SIMULATION MODE (No physical movement)")

        self.gripper_state = SERVO_MIN_US # Start with gripper OPEN

    def deg_to_us(self, degrees):
        """Helper: Converts human-readable Degrees to servo-readable Microseconds."""
        d = max(0, min(SERVO_RANGE_DEG, degrees))
        us = (d / SERVO_RANGE_DEG) * (SERVO_MAX_US - SERVO_MIN_US) + SERVO_MIN_US
        return int(us)

    def send_cmd(self, b, s, e, w, g):
        """
        The Core Motion Function.
        1. Applies Offsets.
        2. Clamps angles to safe limits.
        3. Converts to Microseconds.
        4. Sends string to Arduino.
        """
        if not self.connected: return

        # Apply Offsets & Limits
        b_deg = max(0, min(180, b + OFFSET_BASE))
        s_deg = max(SHLD_MIN, min(SHLD_MAX, s + OFFSET_SHOULDER))
        e_deg = max(ELBW_MIN, min(ELBW_MAX, e + OFFSET_ELBOW))
        w_deg = max(WRST_MIN, min(WRST_MAX, w + OFFSET_WRIST))
        
        # Convert to High-Precision Microseconds
        b_us = self.deg_to_us(b_deg)
        s_us = self.deg_to_us(s_deg)
        e_us = self.deg_to_us(e_deg)
        w_us = self.deg_to_us(w_deg)
        g_us = self.deg_to_us(g) 

        # Send Protocol: "Base,Shld,Elb,Wrist,Grip\n"
        msg = f"{b_us},{s_us},{e_us},{w_us},{g_us}\n"
        self.ser.write(msg.encode())
        
        # Blocking: Wait until Arduino replies "done"
        while True:
            if self.ser.in_waiting:
                line = self.ser.readline().decode().strip()
                if line == "done": break

    def inverse_kinematics(self, x, y, z_height=5.0):
        """
        THE BRAIN: Calculates servo angles to reach (X, Y) at height Z.
        Includes 'Smart Posture' logic to protect the shoulder motor.
        """
        # 1. Base Angle (Simple trigonometry)
        base_angle = math.atan2(y, x) * 180 / math.pi
        r_total = math.sqrt(x**2 + y**2) # Distance from base center
        
        # 2. Tilt Correction Logic
        # Adjusts Z based on distance to fix "sagging" or "hovering"
        raw_correction = (r_total * TILT_SLOPE) + TILT_INTERCEPT
        z_correction = max(MAX_TILT_DROP, raw_correction) # Clamp the drop
        z_final = z_height + z_correction
        
        # 3. Posture Search Loop
        # Instead of calculating one solution, we try multiple wrist angles (pitches).
        # We start looking for a VERTICAL grip (-90 deg) because it's best for chess.
        # If that strains the shoulder, we try flatter angles (up to 0 deg).
        best_solution = None
        
        for pitch in range(-90, 5, 5): 
            result = self.try_reach(r_total, pitch, z_final)
            if result:
                s, e, w = result
                
                # SAFETY CHECK: Is the shoulder upright enough?
                if s >= MIN_SAFE_SHOULDER_ANGLE:
                    return base_angle, s, e, w # Found a safe, valid solution!
                
                # Keep the best "unsafe" solution as a backup just in case
                if best_solution is None or s > best_solution[1]:
                    best_solution = (base_angle, s, e, w)

        if best_solution:
            print(f"[IK] WARNING: Unsafe Shoulder Angle ({best_solution[1]:.1f}°). Moving anyway.")
            return best_solution

        print("[IK] Target Unreachable (Too far or too close).")
        return None

    def try_reach(self, r_total, pitch_deg, z_target):
        """Helper for IK: Solves the triangle for a specific wrist angle."""
        # Calculate where the Wrist Pivot needs to be
        rad_pitch = math.radians(pitch_deg)
        dr = L_WRIST * math.cos(rad_pitch)
        dz = L_WRIST * math.sin(rad_pitch)
        
        r_wrist = r_total - dr
        z_wrist = (z_target - dz) - L_BASE_H 
        
        # Check if arm physically reaches
        h = math.sqrt(r_wrist**2 + z_wrist**2)
        if h > (L_SHOULDER + L_ELBOW): return None 

        # Law of Cosines to solve the 2-Link Arm
        try:
            alpha = math.acos((L_SHOULDER**2 + h**2 - L_ELBOW**2) / (2 * L_SHOULDER * h))
            beta = math.acos((L_SHOULDER**2 + L_ELBOW**2 - h**2) / (2 * L_SHOULDER * L_ELBOW))
        except ValueError: return None 

        theta_line = math.atan2(z_wrist, r_wrist)
        s = (theta_line + alpha) * 180 / math.pi
        e = beta * 180 / math.pi
        
        # Calculate Wrist Servo Angle relative to forearm
        global_forearm = s - (180 - e)
        w = global_forearm - pitch_deg
        return s, e, w

    def move_to_xyz(self, x, y):
        """High-level move command. Default Z=5.0cm to clear chess pieces."""
        angles = self.inverse_kinematics(x, y, z_height=5.0)
        if angles:
            b, s, e, w = angles
            self.send_cmd(b, s, e, w, self.gripper_state)
            return True
        return False
    
    # Simple helpers for gripper state
    def grasp(self): self.gripper_state = 180; pass
    def release(self): self.gripper_state = 0; pass
    #def home(self): self.send_cmd(90, 90, 180, 0, 0)
    # NEW (Updated for safe 600us Wrist and 900us Gripper)
    def home(self): 
        # Base=90, Shoulder=90, Elbow=180, Wrist=18, Gripper=45
        self.send_cmd(90, 90, 180, 18, 45)