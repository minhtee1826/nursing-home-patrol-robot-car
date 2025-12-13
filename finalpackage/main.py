import cv2
import threading
import time
from flask import Flask, render_template, Response
from flask_socketio import SocketIO
from ultralytics import YOLO
from websocket import create_connection
import websocket

# ================= CONFIGURATION =================
# IP of the ESP32 Motor Robot (from your previous code)
ROBOT_WS_URL = "ws://192.168.1.100/ws" 

# IP of the ESP32-CAM (Standard CameraWebServer example)
# Usually port 81 for stream, or just IP depending on code
CAM_STREAM_URL = "http://192.168.1.13:81/stream" 

# ================= SETUP =================
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Load YOLO Model (downloads automatically on first run)
print("Loading YOLO model...")
model = YOLO("yolov8n.pt") 

# Robot WebSocket Connection Holder
robot_ws = None

def connect_to_robot():
    """Tries to connect to the ESP32 Robot WebSocket"""
    global robot_ws
    try:
        robot_ws = create_connection(ROBOT_WS_URL, timeout=2)
        print(f"✅ Connected to Robot at {ROBOT_WS_URL}")
    except Exception as e:
        print(f"⚠️ Could not connect to Robot: {e}")
        robot_ws = None

# Connect initially
connect_to_robot()

# ================= VIDEO PROCESSING =================
def generate_frames():
    cap = cv2.VideoCapture(CAM_STREAM_URL)
    
    while True:
        success, frame = cap.read()
        if not success:
            # If camera disconnects, try to reconnect or send blank
            cap.open(CAM_STREAM_URL)
            time.sleep(0.5)
            continue

        # --- YOLO DETECTION ---
        # Run inference
        results = model(frame, stream=True)
        
        # Plot results on the frame
        for r in results:
            # Check for specific classes if you want (e.g., metals/bottles)
            # You can access r.boxes.cls to filter
            
            # Draw bounding boxes
            frame = r.plot()
            
            # OPTIONAL: Check if 'bottle' or 'cup' is detected and alert frontend
            # This logic would go here to emit socketio events
            
        # --- ENCODE AND STREAM ---
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# ================= FLASK ROUTES =================
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    # Returns the MJPEG stream of the YOLO-processed video
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# ================= WEBSOCKET COMMANDS (Browser -> PC -> Robot) =================
@socketio.on('command')
def handle_command(cmd):
    """Receives command from Browser, forwards to Robot"""
    global robot_ws
    print(f"Received command: {cmd}")
    
    # 1. Forward to Robot
    if robot_ws:
        try:
            robot_ws.send(cmd)
        except (websocket.WebSocketConnectionClosedException, BrokenPipeError):
            print("Robot disconnected, trying to reconnect...")
            connect_to_robot()
    else:
        # Try to connect if not connected
        connect_to_robot()
        if robot_ws: robot_ws.send(cmd)

@socketio.on('connect')
def test_connect():
    print('Client (Browser) connected to Python Server')

# ================= MAIN =================
if __name__ == '__main__':
    # Host 0.0.0.0 allows phones on the same WiFi to access this PC
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)