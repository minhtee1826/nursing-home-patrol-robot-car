import cv2
import time
import threading
from flask import Flask, render_template, Response, request
from flask_socketio import SocketIO, emit
from ultralytics import YOLO

# ================= CONFIGURATION =================
# Default ESP32-CAM IP (can be changed via web interface)
CAM_IP = "192.168.2.212"  
STREAM_URL = f"http://{CAM_IP}:81/stream"

# Detection confidence threshold for alerts
CONFIDENCE_THRESHOLD = 0.8

# Cooldown to prevent spam (seconds between alerts)
ALERT_COOLDOWN = 2.0
last_alert_time = 0

# Camera control
camera_lock = threading.Lock()
current_cap = None

# ================= SETUP =================
app = Flask(__name__)
app.config['SECRET_KEY'] = 'robot-detection-secret'
socketio = SocketIO(app, cors_allowed_origins="*")

# Load YOLO Model (Nano version for speed)
model = YOLO("my_model.pt") 

print(f"Connecting to Camera at: {STREAM_URL}")
print(f"Detection alert threshold: {CONFIDENCE_THRESHOLD * 100}%")

def generate_frames():
    """
    Reads camera frames, runs YOLO, and yields them as a MJPEG stream.
    """
    global last_alert_time, current_cap, STREAM_URL
    
    current_cap = cv2.VideoCapture(STREAM_URL)
    
    # Check connection immediately
    if not current_cap.isOpened():
        print("❌ Error: Could not reach ESP32-CAM. Check IP and Power.")

    while True:
        with camera_lock:
            if current_cap is None:
                current_cap = cv2.VideoCapture(STREAM_URL)
            
            success, frame = current_cap.read()
        
        if not success:
            # If camera disconnects, try to reconnect loop
            with camera_lock:
                current_cap.open(STREAM_URL)
            time.sleep(0.5)
            continue

        # 1. RUN YOLO AI
        results = model(frame, stream=True, verbose=False)

        # 2. PROCESS RESULTS AND DRAW BOXES
        for r in results:
            frame = r.plot()
            
            # Check for high-confidence detections
            if r.boxes is not None and len(r.boxes) > 0:
                for box in r.boxes:
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    class_name = model.names[class_id]
                    
                    # Send alert if confidence > threshold and cooldown passed
                    current_time = time.time()
                    if confidence >= CONFIDENCE_THRESHOLD and (current_time - last_alert_time) > ALERT_COOLDOWN:
                        last_alert_time = current_time
                        confidence_percent = round(confidence * 100, 1)
                        
                        # Emit alert to connected clients
                        socketio.emit('metal_alert', {
                            'detected': True,
                            'class': class_name,
                            'confidence': confidence_percent
                        })
                        print(f"🎯 Detection: {class_name} ({confidence_percent}%)")

        # 3. ENCODE TO JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        # 4. YIELD FRAME (Standard MJPEG format)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# ================= ROUTES =================
@app.route('/')
def index():
    """Serves the HTML page"""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """The src for the <img> tag"""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/get_camera_ip')
def get_camera_ip():
    """Returns current camera IP"""
    return {'ip': CAM_IP}

# ================= SOCKET.IO EVENTS =================
@socketio.on('connect')
def handle_connect():
    print("🔌 Client connected")
    emit('camera_ip', {'ip': CAM_IP})

@socketio.on('disconnect')
def handle_disconnect():
    print("❌ Client disconnected")

@socketio.on('set_camera_ip')
def handle_set_camera_ip(data):
    """Handle camera IP change request from client"""
    global CAM_IP, STREAM_URL, current_cap
    
    new_ip = data.get('ip', '').strip()
    if not new_ip:
        emit('camera_status', {'status': 'error', 'message': 'IP không hợp lệ'})
        return
    
    print(f"📷 Changing camera IP to: {new_ip}")
    
    # Update global variables
    CAM_IP = new_ip
    STREAM_URL = f"http://{CAM_IP}:81/stream"
    
    # Close existing camera connection
    with camera_lock:
        if current_cap is not None:
            current_cap.release()
            current_cap = cv2.VideoCapture(STREAM_URL)
    
    emit('camera_status', {'status': 'success', 'ip': CAM_IP})
    print(f"✅ Camera IP updated to: {CAM_IP}")

# ================= RUN SERVER =================
if __name__ == '__main__':
    # Use socketio.run instead of app.run for WebSocket support
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)