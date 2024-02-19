import serial
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5.QtGui import *
from PyQt5 import *

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.UI()

        self.window_2 = MotorWindow()
        self.window_3 = ArmWindow()


    def UI(self):
        self.title = QLabel('Choose between motor and arm.', self)    
        self.setWindowTitle("Motor and Arm Gui")

        buttonMotors = QPushButton('Motors',self)
        buttonArm = QPushButton('Arm', self)

        buttonMotors.clicked.connect(self.motor_on_click)
        buttonArm.clicked.connect(self.arm_on_click)

        self.setGeometry(1600,0,600,300)  

        
        self.title.setFont(QFont('Arial', 20))
        buttonMotors.resize(150, 50)
        buttonArm.resize(150,50)

        self.title.move(100,25)
        buttonMotors.move(50, 200)
        buttonArm.move(400, 200)

    def motor_on_click(self):
        print("motor clicked")
        self.window_2.show()
        


    def arm_on_click(self):
        print("arm clicked")
        self.window_3.show()



commPort = '/dev/ttyACM0'
serMotor = serial.Serial(commPort, baudrate = 115200)

class MotorWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.UI_Motor()

    def UI_Motor(self):
        self.setWindowTitle("Motor Control")
        self.setGeometry(1600,430,600,350)

        self.text = QLabel('Fowards = W || Backwards = S || Right = D || Left = A', self)  
        self.text2 = QLabel('Speed Up = Shift || Stop = Escape', self)
        self.text.setFont(QFont('Arial', 15))      
        self.text.move(60,50)
        self.text2.setFont(QFont('Arial', 15))      
        self.text2.move(150,150)

        closeButton = QPushButton('Close', self)
        closeButton.clicked.connect(self.close_on_click)
        closeButton.resize(150, 50)
        closeButton.move(225,250)

    def close_on_click(self):
        self.close()

    def keyPressEvent(self, event):
        if(event.key() == Qt.Key_W):
            print('Foward')
            self.foward()
        if(event.key() == Qt.Key_S):
            print("backwards")
            self.backward()     
        if(event.key() == Qt.Key_A):
            print('Left')
            self.right()
        if(event.key() == Qt.Key_D):
            print('Right')
            self.left()
        if(event.key() == Qt.Key_Shift):
            print('Speed Up')
            self.speed_up()
        if(event.key() == Qt.Key_Escape):
            print('stop')
            self.stop()

    def foward(self):
        serMotor.write(bytes(b'foward'))
    
    def backward(self):
        serMotor.write(bytes(b'backward'))
    
    def left(self):
        serMotor.write(bytes(b'left'))
    
    def right(self):
        serMotor.write(bytes(b'right'))
    
    def speed_up(self):
        serMotor.write(bytes(b'boost'))

    def stop(self):
        serMotor.write(bytes(b'stop'))

       

commPort2 = '/dev/ttyACM1'
serArm = serial.Serial(commPort2, baudrate = 115200)
class ArmWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.UI_Arm()

    def UI_Arm(self):
        self.setWindowTitle("Arm Control")
        self.setGeometry(1600,950,600,350)

        self.text = QLabel('Claw Opened = X || Claw Closed = C', self)  
        self.text2 = QLabel('Base Shift Right = D || Base Shift Left = A', self)
        self.text3 = QLabel('Fowards:', self)
        self.text4 = QLabel('Bottom Joint = U || Middle Joint = I || Top Joint = O', self)
        self.text5 = QLabel('Backwards: ', self)
        self.text6 = QLabel('Bottom Joint = J || Middle Joint = K || Top Joint = L', self)
        self.text7 = QLabel('Stop All = Escape', self)
        self.text.setFont(QFont('Arial', 15))      
        self.text.move(50,25)
        self.text2.setFont(QFont('Arial', 15))      
        self.text2.move(50,75)
        self.text3.setFont(QFont('Arial', 15))      
        self.text3.move(50,120)
        self.text4.setFont(QFont('Arial', 13))      
        self.text4.move(50,140)
        self.text5.setFont(QFont('Arial', 15))      
        self.text5.move(50,170)
        self.text6.setFont(QFont('Arial', 13))      
        self.text6.move(50,190)
        self.text7.setFont(QFont('Arial', 15))      
        self.text7.move(50,230)

        closeButton = QPushButton('Close', self)
        closeButton.clicked.connect(self.close_on_click)
        closeButton.resize(150, 50)
        closeButton.move(225,275)

    def close_on_click(self):
        self.close()
 
    def keyPressEvent(self, event):
        if(event.key() == Qt.Key_X):
            print('Claw Opened')
            self.claw_open()
        if(event.key() == Qt.Key_C):
            print("Claw Closed")
            self.claw_closed()     
        if(event.key() == Qt.Key_D):
            print('Base Shift Right')
            self.base_right()
        if(event.key() == Qt.Key_A):
            print('Bace Shift Left')
            self.base_left()
        if(event.key() == Qt.Key_U):
            print('Bottom Joint Foward')
            self.bottom_joint_foward()
        if(event.key() == Qt.Key_J):
            print('Bottom Joint Backwards')
            self.bottom_joint_backwards()
        if(event.key() == Qt.Key_I):
            print('Middle Joint Foward')
            self.middle_joint_foward()
        if(event.key() == Qt.Key_K):
            print('Middle Joint Backwards')
            self.middle_joint_backwards()
        if(event.key() == Qt.Key_O):
            print('Middle Joint Foward')
            self.top_joint_foward()
        if(event.key() == Qt.Key_L):
            print('Middle Joint Backwards')
            self.top_joint_backwards()
        if(event.key() == Qt.Key_Escape):
            print('Stop All')
            self.stop_all()

    def claw_open(self):
        serArm.write(bytes(b'Claw Open'))
    
    def claw_closed(self):
        serArm.write(bytes(b'Claw Closed'))
    
    def base_right(self):
        serArm.write(bytes(b'Base Shift Right'))
    
    def base_left(self):
        serArm.write(bytes(b'Base Shift Left'))
    
    def bottom_joint_foward(self):
        serArm.write(bytes(b'Bottom Joint Foward'))

    def bottom_joint_backwards(self):
        serArm.write(bytes(b'Bottom Joint Backwards'))

    def middle_joint_foward(self):
        serArm.write(bytes(b'Middle Joint Foward'))

    def middle_joint_backwards(self):
        serArm.write(bytes(b'Middle Joint Backwards'))

    def top_joint_foward(self):
        serArm.write(bytes(b'Top Joint Foward'))

    def top_joint_backwards(self):
        serArm.write(bytes(b'Top Joint Backwards'))
    
    def stop_all(self):
        serArm.write(bytes(b'Stop All'))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    main = MainWindow()
    main.show()

    sys.exit(app.exec())

