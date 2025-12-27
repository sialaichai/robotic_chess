♟️ RobotArm-Chess
An autonomous robotic arm that plays physical chess against a human opponent using Computer Vision.

This project provides a complete software stack for a 4-DOF (Degree of Freedom) robotic arm. It uses a webcam to "see" the board, Stockfish to calculate the best moves, and Inverse Kinematics to physically move pieces.

✨ Features
Computer Vision (No E-Board Required): Uses standard image differencing to detect moves. You can play on any standard chessboard.

Smart Motion Planning:

Inverse Kinematics (IK): Calculates servo angles (Base, Shoulder, Elbow, Wrist) based on XYZ coordinates.

"Square Wave" Paths: Pieces are lifted vertically, moved horizontally, and lowered precisely to avoid knocking over other pieces.

Unified Motion Class: A central "Brain" handles complex logic like capturing (moving victim to trash, then moving attacker) and Castling.

Full GUI Control: A Tkinter-based interface for jogging the robot, testing specific squares, and running the game.

Robust Game Logic: Handles special chess rules:

En Passant

Castling

Pawn Promotion (pauses for human intervention)

Dual-Profile Calibration: Separate calibration profiles for playing as White or Black.

🛠️ Hardware Requirements
Robotic Arm: 4-DOF Arm + Gripper (Base, Shoulder, Elbow, Wrist, Grip).

Microcontroller: Arduino (or similar) connected via USB Serial.

Camera: Standard USB Webcam (mounted overhead for a top-down view).

Chess Set: Standard physical board and pieces (high contrast pieces work best).

Lighting: Consistent overhead lighting is critical for the vision system.

📦 Software Prerequisites
1. Python Libraries
Install the required dependencies via pip:

Bash

pip install opencv-python numpy python-chess pyserial
(Note: tkinter is usually included with Python installations)

2. Stockfish Engine
You must download the Stockfish chess engine binary.

Download it from stockfishchess.org.

Place the .exe file in the project root.

Important: Update the STOCKFISH_PATH variable in the code to match your filename:

Python

STOCKFISH_PATH = "stockfish_15_x64_avx2.exe"
⚙️ Calibration Guide (The Workflow)
Before playing, the robot must learn the physical space of the board.

Step 1: Physical Calibration
Run the script: python chess_robot.py.

Go to Tab 1: Jog & Calib.

Select "Edit WHITE Profile".

Use the WASD/IJKL buttons to manually move the robot tip to the center of square A1.

Click "Save A1".

Repeat for H1, H8, and A8.

Click "Save Calibration File".

Fine-tuning: After calibrating the 4 corners A1, A8, H1, H8, use the goto function to fine tune the
location of other squares. Pin down the remaining squares once they are accurate located using hte Jog function.

Note: DO NOT pin A1, A8, H1 and H8.

Step 2: Vision Calibration
Go to Tab 3: Vision Game.

Click "Start Camera Stream".

A window will pop up showing the camera feed.

Click the exact center of the A8 square on the video feed, then click the "A8" button in the GUI to assign it.

Repeat for H8, A1, and H1.

Click "Save & Close".

🎮 How to Play
Select Mode: In Tab 3, choose whether the Robot is White (moves first) or Black.

Start: Click "START GAME".

The Loop:

Robot Turn: The robot thinks (via Stockfish), calculates the path, and physically moves the piece.

Human Turn:

Make your move on the board.

Ensure your hand is out of the frame.

Press SPACEBAR to scan.

The Vision system will propose a move (e.g., "Verify: e2e4?").

Press ENTER to confirm or ESC to cancel/retry. (Vision may compute the wrong moves)

Press R to reset if the camera did not capture any legal move.

Stopping: Press the red "STOP GAME & PARK" button to safely freeze the robot.

📁 File Structure
The script is designed as a "Monolith" (single file) for portability, but logically divided into 5 sections:

Section 1: Imports & Config: Constants for Serial ports, servo limits, and dimensions.

Section 2: Hardware Driver: ChessRobotArm class handling Serial comms and Inverse Kinematics.

Section 3: Logic & Mapping: BoardMapper (coordinate interpolation) and RobotMotion (high-level move sequences).

Section 4: Vision & Loop: OpenCV processing and the main Game Loop state machine.

Section 5: GUI: Tkinter interface.

⚠️ Troubleshooting
Lighting Issues: If the vision system detects moves incorrectly, ensure lighting is even and there are no shadows falling across multiple squares.

Grip Crash: If the gripper hits pieces, adjust the Z_HOVER and Z_DROP heights in the RobotMotion class.

Serial Error: Ensure no other software (like Arduino IDE) is using the same COM when you launch the script.

🤝 Contributing
Feel free to fork this project and improve the Inverse Kinematics solver or add a stronger vision model (like YOLO) for piece recognition!
