import serial
import time

# --- CONFIG ---
PORT = 'COM8'  # Check your Device Manager if this is different
BAUD = 9600

def send_grip(ser, angle):
    # Sends a command to keep the arm steady but change ONLY the gripper
    # B90, S90, E180, W18 (Standard Home Position)
    # Convert angle to microseconds (400-2400 scale)
    pw = int((angle / 180.0) * (2400 - 400) + 400)
    
    # Home Pose + Your Test Grip
    # Note: We hardcode the arm to "Home" so it doesn't move wildly
    cmd = f"1550,1400,1289,1289,{pw}\n"
    ser.write(cmd.encode())
    print(f"   -> Sent Angle {angle} (Pulse: {pw})")

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(3) # Wait for Arduino restart
        print(f"Connected to {PORT}. Ready to test.")
    except:
        print(f"Could not connect to {PORT}. Close Robotchess and try again.")
        return

    print("========================================")
    print("      LIVE GRIP TUNER")
    print("========================================")
    print("1. Place a Knight in the gripper.")
    print("2. Type an angle (e.g. 150).")
    print("3. Wait 3 seconds. If it goes 'limp', the angle is too high.")
    print("   If it slips, the angle is too low.")
    print("========================================")

    while True:
        val = input("\nEnter Angle (120-180) or 'q' to quit: ")
        if val.lower() == 'q': break
        
        try:
            angle = float(val)
            send_grip(ser, angle)
            print("   -> HOLDING... Watch for 3 seconds...")
            time.sleep(3)
            print("   -> If it didn't relax, this angle is SAFE.")
        except ValueError:
            print("Please enter a number.")

    ser.close()

if __name__ == "__main__":
    main()