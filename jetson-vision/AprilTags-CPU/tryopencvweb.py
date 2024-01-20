# april tag detector 
import cv2
import numpy as np
from time import time, sleep
# import apriltag
import pupil_apriltags as apriltag # for windows

cap = cv2.VideoCapture(0) # camera port
cap.set(3, 640) # width
cap.set(4, 480) # height
# detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9', nthreads=1))
detector = apriltag.Detector(families='tag36h11', nthreads=4) # for windows

running = True
frame_bytes = None
import threading
frame_event = threading.Event()
def generate_frames():
    global frame_bytes
    t = time()
    print("generate frames activated")
    while running:
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        tags = detector.detect(gray)

        for tag in tags:
            corners_pos = []
            for corner in tag.corners:
                corners_pos.append(tuple(corner.astype(int)))
                cv2.circle(frame, corners_pos[-1], 4, (255,0,0), 2)
            for side in range(4):
                cv2.line(frame, corners_pos[side-1], corners_pos[side], (0, 255, 0), 3)
        fps = 1/(time()-t)
        t = time()
        # print(f"fps: {fps}") 

        # Convert the frame to bytes
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        frame_event.set()

        print("frame_bytes in generate_frames() is None:", frame_bytes is None)

detection_thread = threading.Thread(target=generate_frames)
detection_thread.start()

camera_stream_update_rate = 24
def get_frames():
    while running:
        frame_event.wait()
        frame_event.clear()

        print("frame_bytes in get_frames() is None:", frame_bytes is None)

        if frame_bytes is None:
            # If frame_bytes is None, load and yield a restored picture
            with open('./no_result.png', 'rb') as image_file:
                restored_image_bytes = image_file.read()
            yield (b'--frame\r\n'
                   b'Content-Type: image/png\r\n\r\n' + restored_image_bytes + b'\r\n')
        else:
            # Yield the regular frame
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


from flask import Flask, render_template, Response
app = Flask(__name__)
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(get_frames(), mimetype='multipart/x-mixed-replace; boundary=frame') 

try:
    app.run(host='0.0.0.0', port=8889, debug=True)
except KeyboardInterrupt:
    pass
running = False
print("shutdown by user")