import socket
import sys
import json

class SocketClient():

    def __init__(self, host, port) -> None:
        self.host = host
        self.port = port
        self.data = ""
        self.received = ""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    def send(self, dict_message):
        try:
            # Connect to server and send data
            self.sock.connect((self.host, self.port))
            self.sock.send(bytes(json.dumps(dict_message), 'UTF-8'))
            # Receive data from the server and shut down
            self.received = self.sock.recv(1024)

        finally:
            self.sock.close()
        self._output()
    
    def _output(self):
        print (f"Sent: {self.data}")
        print (f"Received: {self.received}")