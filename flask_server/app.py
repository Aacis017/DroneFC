"""
Flask Server - NippleJS Joystick Control for DroneFC

This shows how to send joystick data from NippleJS to the Arduino drone.
"""

import serial
import json
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO
import threading
import time

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# UART connection to Arduino
SERIAL_PORT = '/dev/serial0'  # Raspberry Pi UART
BAUD_RATE = 115200

ser = None

def init_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"[DEBUG] Serial connected successfully: {SERIAL_PORT} at {BAUD_RATE} baud")
        return True
    except serial.SerialException as e:
        print(f"[DEBUG] Serial port error: {e}")
        print(f"[DEBUG] Make sure Arduino is connected and port {SERIAL_PORT} is correct")
        socketio.emit('arduino_error', {'message': str(e), 'type': 'serial_init'})
        return False
    except Exception as e:
        print(f"[DEBUG] Unexpected serial error: {e}")
        return False

def send_command(cmd_dict):
    """Send JSON command to Arduino (no response wait)"""
    global ser
    if ser and ser.is_open:
        try:
            cmd_str = json.dumps(cmd_dict) + '\n'
            print(f"[DEBUG] TX -> Arduino: {cmd_str.strip()}")
            ser.write(cmd_str.encode())
            return True
        except serial.SerialException as e:
            print(f"[DEBUG] Serial write error: {e}")
            socketio.emit('arduino_error', {'message': str(e), 'type': 'send_failed'})
            return False
        except Exception as e:
            print(f"[DEBUG] Send error: {e}")
            return False
    else:
        print(f"[DEBUG] Cannot send - Serial not connected")
        socketio.emit('arduino_error', {'message': 'Serial not connected', 'type': 'not_connected'})
        return False

def send_command_with_response(cmd_dict, timeout=1.0):
    """Send JSON command to Arduino and wait for response"""
    global ser
    if not ser or not ser.is_open:
        print(f"[DEBUG] Cannot send - Serial not connected")
        return {"status": "error", "msg": "Serial not connected", "arduino_responded": False}
    
    try:
        # Clear any pending data
        ser.reset_input_buffer()
        
        # Send command
        cmd_str = json.dumps(cmd_dict) + '\n'
        print(f"[DEBUG] TX -> Arduino: {cmd_str.strip()}")
        ser.write(cmd_str.encode())
        
        # Wait for response with timeout
        start_time = time.time()
        response_line = ""
        
        while (time.time() - start_time) < timeout:
            if ser.in_waiting:
                line = ser.readline().decode().strip()
                if line:
                    print(f"[DEBUG] RX <- Arduino: {line}")
                    try:
                        response = json.loads(line)
                        # Check if this is a command response (not telemetry)
                        if response.get('type') != 'telemetry':
                            response['arduino_responded'] = True
                            return response
                    except json.JSONDecodeError:
                        print(f"[DEBUG] Non-JSON response: {line}")
            time.sleep(0.01)
        
        # Timeout - no response received
        print(f"[DEBUG] Arduino response TIMEOUT after {timeout}s")
        return {"status": "timeout", "msg": "Arduino did not respond", "arduino_responded": False}
        
    except serial.SerialException as e:
        print(f"[DEBUG] Serial error: {e}")
        return {"status": "error", "msg": str(e), "arduino_responded": False}
    except Exception as e:
        print(f"[DEBUG] Error: {e}")
        return {"status": "error", "msg": str(e), "arduino_responded": False}

# ========== ROUTES ==========

@app.route('/')
def index():
    return render_template('joystick.html')

@app.route('/arm', methods=['POST'])
def arm():
    print("[DEBUG] ARM command received from browser")
    response = send_command_with_response({"cmd": "arm"})
    print(f"[DEBUG] Arduino ARM response: {response}")
    return jsonify(response)

@app.route('/disarm', methods=['POST'])
def disarm():
    print("[DEBUG] DISARM command received from browser")
    response = send_command_with_response({"cmd": "disarm"})
    print(f"[DEBUG] Arduino DISARM response: {response}")
    return jsonify(response)

@app.route('/takeoff', methods=['POST'])
def takeoff():
    alt = request.json.get('altitude', 1.0)
    print(f"[DEBUG] TAKEOFF command received, altitude: {alt}m")
    response = send_command_with_response({"cmd": "takeoff", "alt": alt})
    print(f"[DEBUG] Arduino TAKEOFF response: {response}")
    return jsonify(response)

@app.route('/land', methods=['POST'])
def land():
    print("[DEBUG] LAND command received from browser")
    response = send_command_with_response({"cmd": "land"})
    print(f"[DEBUG] Arduino LAND response: {response}")
    return jsonify(response)

# ========== SOCKET.IO for real-time joystick ==========

@socketio.on('joystick')
def handle_joystick(data):
    """
    Receive joystick data from NippleJS
    
    Expected data format:
    {
        "throttle": 1000-2000,
        "roll": -30 to 30,
        "pitch": -30 to 30,
        "yaw": -180 to 180
    }
    """
    cmd = {
        "cmd": "rc",
        "t": int(data.get('throttle', 1000)),
        "r": float(data.get('roll', 0)),
        "p": float(data.get('pitch', 0)),
        "y": float(data.get('yaw', 0))
    }
    send_command(cmd)

# ========== TELEMETRY READER ==========

def telemetry_thread():
    """Read telemetry from Arduino and broadcast to clients"""
    global ser
    print("[DEBUG] Telemetry thread started")
    while True:
        if ser and ser.is_open:
            try:
                if ser.in_waiting:
                    line = ser.readline().decode().strip()
                    if line:
                        print(f"[DEBUG] RX <- Arduino: {line}")
                        try:
                            data = json.loads(line)
                            socketio.emit('telemetry', data)
                        except json.JSONDecodeError as e:
                            print(f"[DEBUG] JSON parse error: {e} - Raw: {line}")
            except serial.SerialException as e:
                print(f"[DEBUG] Serial read error: {e}")
                socketio.emit('arduino_error', {'message': str(e), 'type': 'read_error'})
            except Exception as e:
                print(f"[DEBUG] Telemetry error: {e}")
        time.sleep(0.05)  # 20Hz telemetry

# ========== MAIN ==========

if __name__ == '__main__':
    init_serial()
    
    # Start telemetry thread
    t = threading.Thread(target=telemetry_thread, daemon=True)
    t.start()
    
    # Run Flask
    socketio.run(app, host='0.0.0.0', port=5000)
