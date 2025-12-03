import chess
import chess.engine
import time
import sys

# --- CONFIGURATION ---
# Ensure this path points to your actual stockfish.exe
STOCKFISH_PATH = r"stockfish/stockfish-windows-2022-x86-64-avx2.exe"

class BoardMapper:
    def __init__(self, calibration_data):
        # Data format: (Side_X, Fwd_Y)
        self.a1 = calibration_data['a1']
        self.h8 = calibration_data['h8']
        
        # Calculate step size per rank/file
        # X is Side (Columns a-h)
        # Y is Forward (Rows 1-8)
        self.x_step = (self.h8[0] - self.a1[0]) / 7.0
        self.y_step = (self.h8[1] - self.a1[1]) / 7.0
        
    def get_coords(self, square_name):
        # Map 'e2' to (x, y)
        col_char = square_name[0] # a-h (Side/X)
        row_char = square_name[1] # 1-8 (Forward/Y)
        
        col_idx = ord(col_char) - ord('a') # 0-7
        row_idx = int(row_char) - 1        # 0-7
        
        # Calculate Target
        # X (Side) depends on Column
        # Y (Forward) depends on Row
        x = self.a1[0] + (col_idx * self.x_step)
        y = self.a1[1] + (row_idx * self.y_step)
        
        return x, y

def play_game(robot, mapper):
    try:
        engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH_PATH)
    except FileNotFoundError:
        print(f"Stockfish not found at {STOCKFISH_PATH}")
        return

    board = chess.Board()
    print("\n--- GAME START ---")
    print("You are White. Enter moves (e.g. 'e2e4').")

    while not board.is_game_over():
        print("\n" + str(board))
        
        # 1. USER MOVE
        while True:
            user_move = input("Your Move (e.g. e2e4): ")
            try:
                if chess.Move.from_uci(user_move) in board.legal_moves:
                    board.push_uci(user_move)
                    break
                else:
                    print("Illegal move. Try again.")
            except:
                print("Invalid format. Use uci (e.g. e2e4)")

        if board.is_game_over(): break

        # 2. ROBOT MOVE (Engine)
        print("Robot is thinking...")
        result = engine.play(board, chess.engine.Limit(time=0.8))
        bot_move = result.move
        print(f"Robot plays: {bot_move.uci()}")
        
        perform_robot_move(robot, mapper, board, bot_move)
        board.push(bot_move)

    print("Game Over:", board.result())
    engine.quit()

def perform_robot_move(robot, mapper, board, move):
    sq_start = chess.square_name(move.from_square)
    sq_end = chess.square_name(move.to_square)
    
    x1, y1 = mapper.get_coords(sq_start)
    x2, y2 = mapper.get_coords(sq_end)

    # 1. Handle Capture (Remove enemy piece first)
    if board.is_capture(move):
        print(f"[Robot] Capturing piece at {sq_end}")
        robot.release()
        robot.move_to_xyz(x2, y2) # Go to enemy
        time.sleep(0.5)
        robot.grasp()
        
        # Move to "Bin" (Dump zone on the left)
        robot.send_cmd(180, 90, 90, 90, 180) 
        time.sleep(0.5)
        
        robot.release()
        robot.home()

    # 2. Move Own Piece
    print(f"[Robot] Moving {sq_start} to {sq_end}")
    
    # A. Go to Start
    robot.release()
    robot.move_to_xyz(x1, y1)
    time.sleep(1.0)
    
    # B. Grasp
    robot.grasp()
    robot.move_to_xyz(x1, y1) # Re-send close command
    time.sleep(0.5)
    
    # C. Lift (Safety Hover)
    # Lifts shoulder to 110, Elbow to 160 (Tucked), Wrist 45
    robot.send_cmd(90, 110, 160, 45, 180) 
    time.sleep(0.5)

    # D. Go to Destination
    robot.move_to_xyz(x2, y2)
    time.sleep(1.0)
    
    # E. Release
    robot.release()
    robot.move_to_xyz(x2, y2) # Re-send open command
    time.sleep(0.5)
    
    # F. Return Home
    robot.home()