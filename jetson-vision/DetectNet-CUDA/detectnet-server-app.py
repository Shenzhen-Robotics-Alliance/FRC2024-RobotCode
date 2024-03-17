FLIP_IMG = False
CAM_PORT = 1

from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, cudaAllocMapped, cudaToNumpy, cudaFromNumpy
import cv2
import numpy as np
from time import time, sleep
import threading
from http.server import SimpleHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from time import time, sleep

# display_window = videoOutput("display://0")

server_port = 8889

running = True
frame = cv2.imread("./no_result.png")
new_frame_ready = False
detection_results_ready = False
frame_time_total = 0
frame_time_samplecount = 0
detection_results = "<no results yet>"

camera = cap = cv2.VideoCapture(CAM_PORT, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) # width
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) # height
cap.set(cv2.CAP_PROP_FPS, 30) 
net = detectNet("ssd-mobilenet-v2", model="/home/ironn-maple//NoteDetection-mobilenet.onnx", labels="/home/ironn-maple/Documents/jetson-vision/labels.txt", input_blob="input_0", output_cvg="scores", output_bbox="boxes", threshold=0.92)
# net = detectNet("ssd-mobilenet-v2", threshold=0.8)
def generate_frames():
    global frame, new_frame_ready, detection_results_ready, frame_time_total, frame_time_samplecount, detection_results
    print("<-- DetectNetServer | generate frames activated -->")
    t = time()
    while running:
        print("<-- DetectNetServer | generate frames running -->")
        ret, cvimg = cap.read()
        if cvimg is None:
            continue

        # flip it with opencv
        if (FLIP_IMG):
            cvimg = cv2.rotate(cvimg, cv2.ROTATE_180)
        
        img = cudaFromNumpy(cv2.cvtColor(cvimg, cv2.COLOR_RGB2BGR))

        detections = net.Detect(img, width=640, height=480)
        
        detection_results = ""
        # TODO here, find the object with maximum confident
        maxAreaDetection = None
        maxArea = 0
        for detection in detections:
            if detection.TrackStatus >= 0:  # actively tracking
                center = ( (detection.Left + detection.Right)/2 , (detection.Top + detection.Bottom) / 2 )
                area = (detection.Right - detection.Left) * (detection.Bottom - detection.Top)
                if area > maxArea and detection.ClassID == 1:
                    maxAreaDetection = detection
                    maxArea = area
        
        if maxAreaDetection is not None:
            detection = maxAreaDetection
            center = ( (detection.Left + detection.Right)/2 , (detection.Top + detection.Bottom) / 2 )
            area = (detection.Right - detection.Left) * (detection.Bottom - detection.Top)
            print(f"id:{detection.ClassID}, center:{center}, area:{area}")
            detection_results += f"{detection.ClassID} {center[0]} {center[1]} {area}/"
        else:
            detection_results = "no-rst"
        detection_results += "\n"
        
        raw_frame = cudaToNumpy(img)
        frame = cv2.cvtColor(cv2.resize(raw_frame, (320, 240)), cv2.COLOR_BGR2RGB)
        new_frame_ready = True
        detection_results_ready = True

        frame_time_total += time()-t
        frame_time_samplecount += 1
        t = time()

        # display_window.Render(img) # this can't work if there is line "import cv2"
        print(f"<-- DetectNetServer | detect net running, FPS: {net.GetNetworkFPS()} -->")
        
        
# MJPEG Streaming Server
update_rate = 24
class StreamingHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        global new_frame_ready, detection_results_ready, detection_results, frame_time_samplecount, frame_time_total, detection_results
        print(f"<-- DetectNetServer | client visited {self.path} -->")
        if self.path == "/":
            # Return the main page (index.html)
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            with open('/home/ironn-maple/index.html', 'rb') as f:
                content = f.read()
            self.wfile.write(content)
        elif self.path == '/fps':
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            if frame_time_total == 0:
                self.wfile.write("waiting for camera to start".encode())
            else:
                self.wfile.write( ("FPS: " + str(round(frame_time_samplecount / frame_time_total)))  .encode())
            frame_time_total = 0
            frame_time_samplecount = 0
        elif self.path == '/video_feed':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            while running:
                # while (not new_frame_ready) and running:
                #     print("<-- DetectNetServer | waiting for new frame... -->")
                #     sleep(0.02)
                sleep(1/update_rate)

                if (frame is None):
                    print("<-- DetectNetServer | frame is none -->")
                    continue
                ret, buffer = cv2.imencode('.jpg', frame)
                frame_bytes = buffer.tobytes()
                print("<-- DetectNetServer | feeding frame... -->")
                try:
                    self.send_frame(frame_bytes)
                except (ConnectionResetError, BrokenPipeError):
                    print("client disconnected")
                    return
                new_frame_ready = False
        elif self.path == '/results':
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            while running:
                # while (not detection_results_ready) and running:
                #     print("<-- DetectNetServer | waiting for new results -->")
                #     sleep(0.02)
                sleep(1/update_rate)
                detection_results_ready = False
                if detection_results is None:
                    return
                try:
                    self.wfile.write(detection_results.encode())
                except (ConnectionResetError, ConnectionAbortedError, BrokenPipeError):
                    print("<-- DetectNetServer | client disconnected -->")
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
print("<-- DetectNetServer | starting inspector server... -->")
httpd = ThreadedHTTPServer(('0.0.0.0', server_port), StreamingHandler)
server_thread = threading.Thread(target=httpd.serve_forever)
server_thread.daemon = True
server_thread.start()
print("<-- DetectNetServer |  inspector server started -->")

try:
    generate_frames()
except KeyboardInterrupt:
    pass
running = False
print("<-- DetectNetServer | shutdown by user -->")
httpd.shutdown()
server_thread.join()
