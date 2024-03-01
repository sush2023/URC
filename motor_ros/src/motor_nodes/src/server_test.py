#!/usr/bin/env python3
import socketserver, subprocess, sys
from threading import Thread
from pprint import pprint
import json
import rospy
from std_msgs.msg import UInt8
import time


HOST = '0.0.0.0'
PORT = 999

class Rover():
    def __init__(self):
        self.curr_state = 0
        self.pwm = 0
        self.call_count = 0
        self.name = "rover"
    
    def move(self, state):
        if (state == 6):
            self.curr_state=6
            print("STOPPING!")
            while self.pwm > 0:
                self.pwm -= 5
                time.sleep(0.05)
                print("sending current state")
                talker(self.curr_state)
            talker(self.curr_state)
            print("stopped")
        else:
            if (state != self.curr_state):
                self.call_count = 0
                self.pwm = 0
                self.curr_state = state
                self.call_count += 1
            else: 
                if self.pwm < 200:
                    self.pwm += self.call_count
                print(f"pwm is: {self.pwm}")
        talker(self.curr_state, self.name)

class Arm():
    def __init__(self):
        self.name = "arm"
        self.curr_state = 0
    
    def move(self, state):
        self.curr_state = state
        talker(self.curr_state, self.name)

def talker(data, name):    
    if not rospy.is_shutdown():
        if name == "rover":
            print(f"publishing data:{data} to rover")
            motor_pub.publish(data)
        else:
            print(f"publishing data:{data} to arm")
            arm_pub.publish(data)

class SingleTCPHandler(socketserver.BaseRequestHandler):
    "One instance per connection.  Override handle(self) to customize action."
    def handle(self):
        # self.request is the client connection
        data = self.request.recv(1024)  # clip input at 1Kb
        text = data.decode('utf-8')
        #pprint(json.loads(text))
        #for key in json.loads(text):
        #    pprint(json.loads(text)[key])
        if "rover" in json.loads(text):
            rover.move(json.loads(text)["rover"])
        else:
            arm.move(json.loads(text)["arm"])
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
    rover = Rover()
    arm = Arm()
    motor_pub = rospy.Publisher("motor_controller_publisher", UInt8, queue_size=1000)
    arm_pub = rospy.Publisher("arm_controller_publisher", UInt8, queue_size=1000)
    rospy.init_node("talker", anonymous=True)
    # terminate with Ctrl-C
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        sys.exit(0)
