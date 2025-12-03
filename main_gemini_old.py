import json
import os
import time
import math
from arm import ChessRobotArm
from game import play_game, BoardMapper

CALIBRATION_FILE = "calibration.json"

# --- HELPER: FORWARD KINEMATICS (UPDATED) ---
def get_xy_from_angles(b, s, e, w):
    """
    Calculates the exact (X, Y) of the Gripper Tip.
    """
    # Robot Dimensions (Must match arm.py)
    L1 = 7.0  # Updated L1
    L2 = 17.7
    L3 = 19.5 
    L4 = 17.0 # Updated L4 (Gripper Length)
    
    # 1. Convert to Radians
    rad_b = math.radians(b) 
    rad_s = math.radians(s)
    
    # 2. Global Angles
    global_e_pitch = s - (180 - e)
    rad_e_pitch = math.radians(global_e_pitch)
    
    global_w_pitch = global_e_pitch - w
    rad_w_pitch = math.radians(global_w_pitch)

    # 3. Calculate Horizontal Reach (r)
    r_l2 = L2 * math.cos(rad_s)
    r_l3 = L3 * math.cos(rad_e_pitch)
    r_l4 = L4 * math.cos(rad_w_pitch)
    
    r_total = r_l2 + r_l3 + r_l4
    
    # 4. Convert Polar to Cartesian
    x = r_total * math.cos(rad_b) # Side Distance (Left/Right)
    y = r_total * math.sin(rad_b) # Forward Distance
    
    return x, y

def manual_jog(robot, b, s, e, w, g):
    """Allows user to move robot with keyboard."""
    print("  [JOG MODE] W/S:Shoulder, A/D:Base, I/K:Elbow, J/L:Wrist. 'save' to finish.")
    step = 2
    while True:
        cmd = input("  Jog > ").strip().lower()
        if cmd == 'save': return b, s, e, w, g
        
        # Mapping Keys to Servos
        if cmd == 'w': s += step # Up
        if cmd == 's': s -= step # Down
        if cmd == 'a': b += step # Left
        if cmd == 'd': b -= step # Right
        if cmd == 'i': e += step # Extend
        if cmd == 'k': e -= step # Bend
        if cmd == 'j': w -= step # Down
        if cmd == 'l': w += step # Up
        if cmd == 'o': g = 0     # Open
        if cmd == 'p': g = 180   # Close
        
        # Safety Clamps
        s = max(0, min(130, s))
        e = max(0, min(180, e))
        w = max(0, min(180, w))
        
        robot.send_cmd(b, s, e, w, g)

# --- MENU OPTION 1: CALIBRATION ---
def calibrate_corners(robot):
    print("\n--- NEW CALIBRATION (A1 & H8) ---")
    print("Instructions: Jog the gripper tip to the CENTER of the square.")
    
    corners = []
    # Start Pose
    b, s, e, w, g = 90, 90, 180, 0, 0
    robot.send_cmd(b, s, e, w, g)

    while len(corners) < 2:
        target_name = "A1" if len(corners) == 0 else "H8"
        print(f"\nStep {len(corners)+1}: Move to {target_name}.")
        
        b, s, e, w, g = manual_jog(robot, b, s, e, w, g)
        
        # Calculate Tip Coordinate using ALL angles
        x, y = get_xy_from_angles(b, s, e, w)
        print(f"Captured {target_name}: ({x:.1f}, {y:.1f})")
        corners.append((x, y))

    data = {"a1": corners[0], "h8": corners[1]}
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(data, f)
    
    print("\n[SUCCESS] Calibration saved.")

# --- MENU OPTION 2: TEST & FIX ---
def test_and_fix(robot):
    if not os.path.exists(CALIBRATION_FILE):
        print("\n[ERROR] No calibration found. Run Option 1.")
        return

    with open(CALIBRATION_FILE, 'r') as f:
        data = json.load(f)
    
    mapper = BoardMapper(data)
    
    print("\n--- TEST & FIX MODE ---")
    print("Type a square (e.g. 'e4'). If wrong, type 'fix'.")
    
    b, s, e, w, g = 90, 90, 180, 0, 0 
    last_target_xy = None 

    while True:
        val = input("\nTest Square > ").strip().lower()
        if val in ['done', 'q', 'exit']: break
        
        # --- FIX LOGIC ---
        if val == 'fix':
            if last_target_xy is None:
                print("  [Error] No move to fix.")
                continue

            print("  [FIX MODE] Jog robot to the TRUE position.")
            b, s, e, w, g = manual_jog(robot, b, s, e, w, g)
            
            real_x, real_y = get_xy_from_angles(b, s, e, w)
            target_x, target_y = last_target_xy
            
            dx = real_x - target_x
            dy = real_y - target_y
            
            print(f"  [ADJUST] Shifting grid X:{dx:.2f}, Y:{dy:.2f}")
            
            data['a1'] = (data['a1'][0] + dx, data['a1'][1] + dy)
            data['h8'] = (data['h8'][0] + dx, data['h8'][1] + dy)
            
            with open(CALIBRATION_FILE, "w") as f:
                json.dump(data, f)
            
            mapper = BoardMapper(data)
            print("  [SAVED] Grid updated.")
            continue
        # -----------------

        try:
            tx, ty = mapper.get_coords(val)
            last_target_xy = (tx, ty) 
            
            print(f"Moving to {val} -> ({tx:.1f}, {ty:.1f})")
            
            # Use Closed Gripper (Pointer Mode)
            robot.gripper_state = 180

            robot.home()
            time.sleep(0.5)
            robot.move_to_xyz(tx, ty)
            
            # Update local tracking angles for next manual jog
            # Use default safe Z=3.0 for testing
            angles = robot.inverse_kinematics(tx, ty, z_height=3.0)
            if angles:
                b, s, e, w = angles
                g = 180
            
        except Exception as e:
            print(f"Error: {e}")

# --- MAIN ---
def main():
    robot = ChessRobotArm(port='COM8') 
    
    while True:
        print("\n=== CHESS ROBOT MENU ===")
        print("1. New Calibration (A1 & H8)")
        print("2. Test & Fix")
        print("3. Play Chess")
        print("4. Exit")
        
        choice = input("Select > ")
        
        if choice == '1':
            calibrate_corners(robot)
        elif choice == '2':
            test_and_fix(robot)
        elif choice == '3':
            if os.path.exists(CALIBRATION_FILE):
                with open(CALIBRATION_FILE, 'r') as f:
                    mapper = BoardMapper(json.load(f))
                play_game(robot, mapper)
            else:
                print("Calibrate first.")
        elif choice == '4':
            break

if __name__ == "__main__":
    main()