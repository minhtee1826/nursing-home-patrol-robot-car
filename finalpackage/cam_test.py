import cv2
import time
import threading
from flask import Flask, render_template, Response
from flask_socketio import SocketIO
from ultralytics import YOLO
from websocket import create_connection
import websocket

# ================= CONFIGURATION =================
ROBOT_IP = "192.168.1.100"       # <--- UPDATE THIS
CAM_IP   = "192.168.1.101"       # <--- UPDATE THIS

ROBOT_WS_URL = f"ws://{ROBOT_IP}/ws"
CAM_STREAM_URL = f"http://{CAM_IP}:81/stream"

# ================= SETUP =================
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")

# Load YOLO
print("⏳ Loading YOLO model...")
model = YOLO("yolov8n.pt") 

# Robot Connection Global Variable
robot_ws = None
robot_connected = False

def connect_to_robot():
    """Background thread to keep Python connected to ESP32"""
    global robot_ws, robot_connected
    while True:
        if not robot_connected:
            try:
                print(f"🔄 Trying to connect to Robot at {ROBOT_WS_URL}...")
                robot_ws = create_connection(ROBOT_WS_URL, timeout=2)
                print("✅ Connected to ESP32 Robot!")
                robot_connected = True
                socketio.emit('robot_status', {'status': 'connected'})
            except Exception as e:
                print(f"⚠️ Robot Connect Fail: {e}")
                robot_connected = False
                socketio.emit('robot_status', {'status': 'disconnected'})
                time.sleep(3) # Wait before retry
        time.sleep(5) # Check health every 5s

# Start Robot Connection Thread
threading.Thread(target=connect_to_robot, daemon=True).start()

# ================= VIDEO GENERATOR =================
def generate_frames():
    cap = cv2.VideoCapture(CAM_STREAM_URL)
    
    # Custom Classes for Detection (e.g., 39=bottle, 67=cell phone for demo)
    # Check coco.names for IDs. Metals are hard for standard YOLO, 
    # so we'll simulate detection logic or train a custom model later.
    
    metal_count = 0

    while True:
        success, frame = cap.read()
        if not success:
            cap.open(CAM_STREAM_URL)
            time.sleep(0.5)
            continue

        # Run YOLO
        results = model(frame, stream=True, verbose=False)
        
        detected_objects = []

        for r in results:
            frame = r.plot()
            # Extract class names for logic
            for box in r.boxes:
                cls_id = int(box.cls[0])
                cls_name = model.names[cls_id]
                detected_objects.append(cls_name)

        # Logic: If we see a 'spoon' or 'fork' or 'cell phone', trigger metal alert
        # (Just for demo purposes until you train a custom metal model)
        if 'cell phone' in detected_objects or 'scissors' in detected_objects:
            socketio.emit('metal_alert', {'detected': True})

        # Encode
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
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# ================= SOCKET COMMANDS =================
@socketio.on('command')
def handle_command(cmd):
    """Receive from Browser -> Send to Robot"""
    global robot_ws, robot_connected
    print(f"📱 Browser sent: {cmd}")
    
    if robot_connected and robot_ws:
        try:
            robot_ws.send(cmd)
        except:
            print("❌ Send failed. Robot disconnected.")
            robot_connected = False
            socketio.emit('robot_status', {'status': 'disconnected'})

@socketio.on('connect')
def client_connect():
    print("💻 Browser Connected")
    # Tell browser the current robot status immediately
    status = 'connected' if robot_connected else 'disconnected'
    socketio.emit('robot_status', {'status': status})

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)