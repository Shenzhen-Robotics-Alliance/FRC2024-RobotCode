'''
inspection and detection server running together
'''
import cv2
import numpy as np
from time import time, sleep
# import apriltag
import pupil_apriltags as apriltag # for windows
import threading
from http.server import SimpleHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from time import time, sleep

cap = cv2.VideoCapture(0) # camera port
cap.set(3, 640) # width
cap.set(4, 480) # height
cap.set(cv2.CAP_PROP_FPS, 30) 
# detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9', nthreads=1))
detector = apriltag.Detector(families='tag36h11', nthreads=4) # for windows

server_port = 8888

running = True
frame = None
new_frame_ready = False
detection_results_ready = False
frame_time_total = 0
frame_time_samplecount = 0
detection_results = "<no results yet>"
import threading
def generate_frames():
    global frame, new_frame_ready, detection_results_ready, frame_time_total, frame_time_samplecount, detection_results
    t = time()
    print("generate frames activated")
    while running:
        dt = time()
        ret, frame = cap.read()
        print("read camera time: " + str(int((time() - dt)*1000)) + "ms")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        dt = time()
        tags = detector.detect(gray)
        print("detector time: " + str(int((time() - dt)*1000)) + "ms")

        dt = time()
        detection_results = ""
        for tag in tags:
            corners_pos = []
            left = top = float("inf")
            right = bottom = 0
            for corner in tag.corners:
                corner_pos = tuple(corner.astype(int))
                corners_pos.append(corner_pos)
                left = min(corner_pos[0], left)
                top = min(corner_pos[1], top)
                right = max(corner_pos[0], right)
                bottom = max(corner_pos[1], bottom)
                cv2.circle(frame, corners_pos[-1], 4, (255,0,0), 2)
            
            for side in range(4):
                cv2.line(frame, corners_pos[side-1], corners_pos[side], (0, 255, 0), 3)
            center = ((corners_pos[0][0] + corners_pos[1][0] + corners_pos[2][0] + corners_pos[3][0]) / 4, (corners_pos[0][1] + corners_pos[1][1] + corners_pos[2][1] + corners_pos[3][1]) / 4)
            area = (right - left) * (bottom - top) # TODO improve this algorithm
            # print(f"id: {tag.tag_id}, center: {center}, area: {area}")
            cv2.putText(frame, f"tag id{tag.tag_id}", (int(center[0]-120), int(center[1])), cv2.FONT_HERSHEY_COMPLEX, 2.0, (100,200,200), 5)
            # detection_results += f"\n{tag.tag_id} {center[0]} {center[1]} {corners_pos[0][0]} {corners_pos[0][1]} {corners_pos[1][0]} {corners_pos[1][1]} {corners_pos[2][0]} {corners_pos[2][1]} {corners_pos[3][0]} {corners_pos[3][1]}"
            detection_results += f"{tag.tag_id} {center[0]} {center[1]} {area}/"
        if detection_results=="":
            detection_results = "no-rst"
        print("process result time: " + str(int((time() - dt)*1000)) + "ms")

        detection_results += "\n"
        frame_time_total += time()-t
        frame_time_samplecount += 1
        t = time()

        # Convert the frame to bytes
        new_frame_ready = True
        detection_results_ready = True

# MJPEG Streaming Server
update_rate = 24
class StreamingHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        global new_frame_ready, detection_results_ready, detection_results, frame_time_samplecount, frame_time_total
        print(f"<--client visited {self.path} -->")
        if self.path == "/":
            # Return the main page (index.html)
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            with open('./index.html', 'rb') as f:
                content = f.read()
            self.wfile.write(content)
        elif self.path == '/fps':
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write( ("FPS: " + str(round(frame_time_samplecount / frame_time_total)))  .encode())
            frame_time_total = 0
            frame_time_samplecount = 0
        elif self.path == '/video_feed':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while running:
                # while (not new_frame_ready) and running:
                #     sleep(0.02)
                sleep(1/update_rate)
                try:
                    frame_resized = cv2.resize(frame, (160, 120))
                    ret, buffer = cv2.imencode('.jpg', frame_resized)
                    frame_bytes = buffer.tobytes()
                    self.send_frame(frame_bytes)
                except ConnectionResetError:
                    print("client disconnected")
                    return
                new_frame_ready = True
        elif self.path == '/results':
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            while running:
                while (not detection_results_ready) and running:
                    sleep(0.05)
                detection_results_ready = False
                try:
                    self.wfile.write(detection_results.encode())
                except (ConnectionResetError, ConnectionAbortedError):
                    print("client disconnected")
                    return
                

    def send_frame(self, frame):
        boundary = b'frame'
        headers = (
            b'Content-Type: image/jpeg\r\n',
            b'Content-Length: ' + str(len(frame)).encode() + b'\r\n\r\n'
        )
        self.wfile.write(b'--' + boundary + b'\r\n' + b''.join(headers) + frame + b'\r\n')

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass

# Start the HTTP server in a separate thread
print("<-- starting inspector server... -->")
httpd = ThreadedHTTPServer(('0.0.0.0', server_port), StreamingHandler)
server_thread = threading.Thread(target=httpd.serve_forever)
server_thread.daemon = True
server_thread.start()
print("<-- inspector server started -->")

try:
    generate_frames()
except KeyboardInterrupt:
    pass
running = False
print("shutdown by user")
httpd.shutdown()
server_thread.join()
