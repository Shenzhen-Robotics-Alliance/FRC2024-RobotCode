from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, cudaAllocMapped, cudaToNumpy
import cv2
import numpy as np
from time import time, sleep
import threading
from http.server import SimpleHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from time import time, sleep

net = detectNet("ssd-mobilenet-v1", model="/home/iron-maple/Documents/jetson-vision/DetectNet-CUDA/mymobilenet.onnx", labels="/home/iron-maple/Documents/jetson-vision/DetectNet-CUDA/labels.txt", input_blob="input_0", output_cvg="scores", output_bbox="boxes", threshold=0.92)
camera = videoSource("/dev/video0") # V4L2 port0
# display_window = videoOutput("display://0")

server_port = 8889

running = True
frame_bytes = None
new_frame_ready = False
detection_results_ready = False
fps = 0
detection_results = "<no results yet>"
import threading
def generate_frames():
    global frame_bytes, new_frame_ready, detection_results_ready, fps, detection_results
    print("generate frames activated")
    while running:
        img = camera.Capture()

        detections = net.Detect(img, width=1280, height=720)
        
        detection_results = ""
        # TODO here, find the object with maximum confident
        for detection in detections:
            if detection.TrackStatus >= 0:  # actively tracking
                center = ( (detection.Left + detection.Right)/2 , (detection.Top + detection.Bottom) / 2 )
                area = (detection.Right - detection.Left) * (detection.Bottom - detection.Top)
                print(f"id:{detection.ClassID}, center:{center}, area:{area}")
                detection_results += f"{detection.ClassID} {center[0]} {center[1]} {area}/"
        if detection_results=="":
            detection_results = "no-rst"
        detection_results += "\n"
        
        raw_frame = cudaToNumpy(img)
        frame = cv2.cvtColor(cv2.resize(raw_frame, (320, 240)), cv2.COLOR_BGR2RGB)
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        new_frame_ready = True
        detection_results_ready = True
        fps = net.GetNetworkFPS()
        # display_window.Render(img) # this can't work if there is line "import cv2"
        # print(f"<-- detect net running, FPS: {net.GetNetworkFPS()} -->")
        
        
# MJPEG Streaming Server
update_rate = 24
class StreamingHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        global new_frame_ready, detection_results_ready, detection_results
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
            self.wfile.write( ("FPS: " + str(fps))  .encode())
        elif self.path == '/video_feed':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while running:
                while (not new_frame_ready) and running:
                    sleep(0.02)
                try:
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
