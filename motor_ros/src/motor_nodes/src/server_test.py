#!/usr/bin/env python3
import socketserver, subprocess, sys
from threading import Thread
from pprint import pprint
import json
import rospy
from std_msgs.msg import UInt8

HOST = '0.0.0.0'
PORT = 999
    
def talker(data):    
    if not rospy.is_shutdown():
        print("publishing to ros node")
        pub.publish(data)

class SingleTCPHandler(socketserver.BaseRequestHandler):
    "One instance per connection.  Override handle(self) to customize action."
    def handle(self):
        # self.request is the client connection
        data = self.request.recv(1024)  # clip input at 1Kb
        text = data.decode('utf-8')
        pprint(json.loads(text))
        for key in json.loads(text):
            pprint(json.loads(text)[key])
        talker(json.loads(text)["rover"])
        self.request.send(bytes(json.dumps({"status":"success!"}), 'UTF-8'))
        self.request.close()

class SimpleServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    # Ctrl-C will cleanly kill all spawned threads
    daemon_threads = True
    # much faster rebinding
    allow_reuse_address = True

    def __init__(self, server_address, RequestHandlerClass):
        socketserver.TCPServer.__init__(self, server_address, RequestHandlerClass)

if __name__ == "__main__":
    server = SimpleServer((HOST, PORT), SingleTCPHandler)
    pub = rospy.Publisher("motor_controller_publisher", UInt8, queue_size=1000)
    rospy.init_node("talker", anonymous=True)
    # terminate with Ctrl-C
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        sys.exit(0)
