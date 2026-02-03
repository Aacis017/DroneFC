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
        print(f"Serial connected: {SERIAL_PORT}")
        return True
    except Exception as e:
        print(f"Serial error: {e}")
        return False

def send_command(cmd_dict):
    """Send JSON command to Arduino"""
    global ser
    if ser and ser.is_open:
        try:
            cmd_str = json.dumps(cmd_dict) + '\n'
            ser.write(cmd_str.encode())
            return True
        except Exception as e:
            print(f"Send error: {e}")
    return False

# ========== ROUTES ==========

@app.route('/')
def index():
    return render_template('joystick.html')

@app.route('/arm', methods=['POST'])
def arm():
    send_command({"cmd": "arm"})
    return jsonify({"status": "ok"})

@app.route('/disarm', methods=['POST'])
def disarm():
    send_command({"cmd": "disarm"})
    return jsonify({"status": "ok"})

@app.route('/takeoff', methods=['POST'])
def takeoff():
    alt = request.json.get('altitude', 1.0)
    send_command({"cmd": "takeoff", "alt": alt})
    return jsonify({"status": "ok"})

@app.route('/land', methods=['POST'])
def land():
    send_command({"cmd": "land"})
    return jsonify({"status": "ok"})

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
    while True:
        if ser and ser.is_open:
            try:
                if ser.in_waiting:
                    line = ser.readline().decode().strip()
                    if line:
                        data = json.loads(line)
                        socketio.emit('telemetry', data)
            except:
                pass
        time.sleep(0.05)  # 20Hz telemetry

# ========== MAIN ==========

if __name__ == '__main__':
    init_serial()
    
    # Start telemetry thread
    t = threading.Thread(target=telemetry_thread, daemon=True)
    t.start()
    
    # Run Flask
    socketio.run(app, host='0.0.0.0', port=5000)
