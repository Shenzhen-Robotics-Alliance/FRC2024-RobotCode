from http.server import BaseHTTPRequestHandler, HTTPServer
import time

hostName = "192.168.10.13"
serverPort = 8888

class ApiServer(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/jetson-vision-api":
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(bytes("hello jetson vision api!!!", "utf-8"))
            return 
        
        self.send_response(404)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        self.wfile.write(bytes("<html><head><title>path incorrect</title></head>", "utf-8"))
        self.wfile.write(bytes("<p>Request: %s</p>" % self.path, "utf-8"))
        self.wfile.write(bytes("<body>", "utf-8"))
        self.wfile.write(bytes("<p>404: page not found</p>", "utf-8"))
        self.wfile.write(bytes("</body></html>", "utf-8"))

webServer = HTTPServer((hostName, serverPort), ApiServer)
print("Server started http://%s:%s" % (hostName, serverPort))

try:
    webServer.serve_forever()
except KeyboardInterrupt:
    webServer.server_close()
    
print("Server stopped.")

