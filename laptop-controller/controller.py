"""
Pendulum Controller with WebSocket - receives commands from web UI.
No keyboard input needed - all control is done through the web interface.
"""
import serial
import time
import subprocess
import os
import json
import threading
import asyncio
import re
import signal
import sys
import queue

# Try to import websockets, install if missing
try:
    import websockets
except ImportError:
    print("Installing websockets...")
    subprocess.check_call(["pip", "install", "websockets"])
    import websockets

PORT = "COM3"           # change this (Arduino IDE shows the port)
BAUD = 115200
BOARD = "arduino:avr:uno"  # change if using a different board
SKETCH_PATH = os.path.join(os.path.dirname(__file__), "..", "arduino-code")
WS_PORT = 8765          # WebSocket server port

# Shared state
current_angle = 0.0
current_velocity = 0.0
current_position = 0
limit_left = False
limit_right = False
connected_clients = set()
state_lock = threading.Lock()
command_queue = queue.Queue()  # Commands from web clients to send to Arduino
ws_message_queue = queue.Queue()  # Messages to broadcast to web clients
running = True

# Serial connection (global for access from multiple threads)
ser = None

def parse_angle_from_line(line):
    """Try to extract angle from Arduino output."""
    match = re.search(r'ang=([0-9.-]+)', line)
    if match:
        return float(match.group(1))
    match = re.search(r'Angle:\s*([0-9.-]+)', line)
    if match:
        return float(match.group(1))
    return None

def parse_velocity_from_line(line):
    """Try to extract velocity from Arduino output."""
    match = re.search(r'vel=([0-9.-]+)', line)
    if match:
        return float(match.group(1))
    return None

def parse_position_from_line(line):
    """Try to extract cart position from Arduino output."""
    match = re.search(r'pos=([0-9.-]+)', line)
    if match:
        return int(float(match.group(1)))
    return None

def parse_limits_from_line(line):
    """Try to extract limit switch status from Arduino output."""
    left_match = re.search(r'limL=([01])', line)
    right_match = re.search(r'limR=([01])', line)
    if left_match and right_match:
        return (left_match.group(1) == '1', right_match.group(1) == '1')
    return None

# WebSocket handler
async def ws_handler(websocket, path=None):
    """Handle WebSocket connections."""
    connected_clients.add(websocket)
    print(f">>> Web client connected ({len(connected_clients)} total)")
    
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                if 'command' in data:
                    cmd = data['command']
                    command_queue.put(cmd)
                    print(f">>> Received command: {cmd}")
            except json.JSONDecodeError:
                pass
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        connected_clients.discard(websocket)
        print(f">>> Web client disconnected ({len(connected_clients)} total)")

async def broadcast_state():
    """Continuously broadcast angle data and other messages to all connected clients."""
    global running
    while running:
        messages_to_send = []
        
        # Check for special messages (like upload status)
        while not ws_message_queue.empty():
            try:
                msg = ws_message_queue.get_nowait()
                messages_to_send.append(msg)
            except:
                break
        
        # Always send state data
        with state_lock:
            angle_data = json.dumps({
                "angle": current_angle,
                "velocity": current_velocity,
                "position": current_position,
                "limitLeft": limit_left,
                "limitRight": limit_right
            })
        messages_to_send.append(angle_data)
        
        if connected_clients:
            disconnected = set()
            for client in connected_clients:
                try:
                    for msg in messages_to_send:
                        await client.send(msg)
                except:
                    disconnected.add(client)
            
            connected_clients.difference_update(disconnected)
        
        await asyncio.sleep(0.033)  # ~30 FPS

async def start_ws_server():
    """Start the WebSocket server."""
    async with websockets.serve(ws_handler, "0.0.0.0", WS_PORT):
        print(f">>> WebSocket server started on ws://localhost:{WS_PORT}")
        await broadcast_state()

def run_ws_server():
    """Run WebSocket server in a separate thread."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(start_ws_server())
    except:
        pass

def upload_arduino_code():
    """Upload Arduino code and return success status."""
    global ser
    
    print("\n>>> Uploading Arduino code...")
    ws_message_queue.put(json.dumps({"upload_status": "started"}))
    
    # Close serial port
    if ser and ser.is_open:
        ser.close()
    
    try:
        result = subprocess.run(
            ["arduino-cli", "compile", "--upload", "-p", PORT, "-b", BOARD, SKETCH_PATH],
            capture_output=True,
            text=True,
            timeout=120  # 2 minute timeout
        )
        
        if result.returncode == 0:
            print(">>> Upload successful!")
            ws_message_queue.put(json.dumps({"upload_status": "success"}))
            success = True
        else:
            print(">>> Upload failed:")
            print(result.stderr)
            ws_message_queue.put(json.dumps({
                "upload_status": "error",
                "message": result.stderr[:200] if result.stderr else "Unknown error"
            }))
            success = False
            
    except FileNotFoundError:
        print(">>> Error: arduino-cli not found")
        ws_message_queue.put(json.dumps({
            "upload_status": "error",
            "message": "arduino-cli not found. Install from arduino.github.io/arduino-cli/"
        }))
        success = False
    except subprocess.TimeoutExpired:
        print(">>> Upload timed out")
        ws_message_queue.put(json.dumps({
            "upload_status": "error",
            "message": "Upload timed out after 2 minutes"
        }))
        success = False
    except Exception as e:
        print(f">>> Upload error: {e}")
        ws_message_queue.put(json.dumps({
            "upload_status": "error",
            "message": str(e)
        }))
        success = False
    
    # Reconnect serial port
    time.sleep(2)  # Wait for Arduino to reset
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        print(f">>> Reconnected to {PORT}")
    except Exception as e:
        print(f">>> Could not reconnect: {e}")
    
    return success

def serial_loop():
    """Main loop: handle serial communication with Arduino."""
    global current_angle, current_velocity, current_position, limit_left, limit_right, ser, running
    
    # Request angle periodically
    last_angle_request = 0
    ANGLE_REQUEST_INTERVAL = 0.1  # Request angle every 100ms
    
    while running:
        try:
            # Process commands from web clients
            while not command_queue.empty():
                cmd = command_queue.get_nowait()
                
                # Handle upload command specially
                if cmd == 'UPLOAD':
                    upload_arduino_code()
                    continue
                
                if ser and ser.is_open:
                    # Config commands (contain ':') need newline
                    if ':' in cmd:
                        ser.write((cmd + '\n').encode())
                    else:
                        ser.write(cmd.encode())
                    print(f">>> Sent to Arduino: {cmd}")
            
            # Periodically request angle
            now = time.time()
            if now - last_angle_request > ANGLE_REQUEST_INTERVAL:
                if ser and ser.is_open:
                    ser.write(b"A")
                last_angle_request = now
            
            # Read responses from Arduino
            if ser and ser.is_open and ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(line)
                    
                    angle = parse_angle_from_line(line)
                    if angle is not None:
                        with state_lock:
                            current_angle = angle
                    
                    velocity = parse_velocity_from_line(line)
                    if velocity is not None:
                        with state_lock:
                            current_velocity = velocity
                    
                    position = parse_position_from_line(line)
                    if position is not None:
                        with state_lock:
                            current_position = position
                    
                    limits = parse_limits_from_line(line)
                    if limits is not None:
                        with state_lock:
                            limit_left = limits[0]
                            limit_right = limits[1]
            
            time.sleep(0.01)
            
        except serial.SerialException as e:
            print(f">>> Serial error: {e}")
            time.sleep(1)
        except Exception as e:
            print(f">>> Error: {e}")
            time.sleep(0.1)

def signal_handler(sig, frame):
    """Handle Ctrl+C."""
    global running
    print("\n>>> Shutting down...")
    running = False
    sys.exit(0)

def main():
    global ser, running
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Start WebSocket server in background
    ws_thread = threading.Thread(target=run_ws_server, daemon=True)
    ws_thread.start()
    
    # Connect to Arduino
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        print(f">>> Connected to Arduino on {PORT}")
    except serial.SerialException as e:
        print(f">>> Error opening {PORT}: {e}")
        print(">>> Running without Arduino (web UI will still work)")
        ser = None
    
    print()
    print("=" * 50)
    print("  Pendulum Controller (Web Interface)")
    print("=" * 50)
    print()
    print(f"  Open in browser: http://localhost:5173")
    print(f"  WebSocket:       ws://localhost:{WS_PORT}")
    print()
    print("  Control the pendulum using the web interface.")
    print("  Press Ctrl+C to stop.")
    print()
    print("=" * 50)
    print()
    
    try:
        serial_loop()
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        if ser and ser.is_open:
            ser.write(b"S")
            ser.close()
        print(">>> Goodbye!")

if __name__ == "__main__":
    main()

