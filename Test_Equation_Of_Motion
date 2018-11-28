import  math


def Split_to_Dict( qt_string: str):
    tokens = qt_string.split(';')
    # delete the last element (it is empty)
    del tokens[len(tokens) - 1]

    Qt_string = {}
    count = 4
    if len(tokens) != count:
        tokens_clone = tokens
        tokens = [""] * count
        for i in range(count):
            tokens[i] = tokens_clone[(len(tokens_clone) - count) + i]

    for term in tokens:
        temp_list = term.split(':')
        Qt_string[temp_list[0]] = int(temp_list[1])

    return Qt_string
def map(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)
class E:
    def __init__(self):
        self.PWMNORMAL = 305
        self.PWMRANGE = 165
        self.ANGLE45 = 0.785398  # 45 deg to rad
        self.ANGLE90 = 1.5708  # 90 deg to rad
        self.ANGLE225 = 3.92699  # 225 deg to rad
        self.FULL_PWM_RANGE_COEFFICIENT = self.PWMRANGE / 100.0  # PWMRANGE/100
        self.MOTORS_BASE_PWM = 305
        self.FULL_ROTATION_COEFFICIENT = 0.3 * 1.65
        self.MAXTHRUST = 0.5
        self.VMAXTHRUST = 0.5
    def calculateHorizontalMotors_17(self,_x,_y,_r):

        theta = math.atan2(_x, _y)
        circle_factor = max(abs(math.cos(theta)), abs(math.sin(theta)))
        resultant = math.hypot(_x, _y) * circle_factor

        # alpha = 45 deg - theta
        # alpha = theta - self.ANGLE225
        alpha = self.ANGLE45 - theta
        maximum_factor = 1 / (math.cos(self.ANGLE45 - abs(theta) + (int(abs(theta) / self.ANGLE90) * self.ANGLE90)))
        RightComponent = resultant * math.cos(alpha) * maximum_factor
        LeftComponent = resultant * math.sin(alpha) * maximum_factor

        front_right_thruster_value = int(self.MOTORS_BASE_PWM + (LeftComponent * self.FULL_PWM_RANGE_COEFFICIENT))
        front_left_thruster_value = int(self.MOTORS_BASE_PWM + (RightComponent * self.FULL_PWM_RANGE_COEFFICIENT))
        back_right_thruster_value = int(self.MOTORS_BASE_PWM + (RightComponent * self.FULL_PWM_RANGE_COEFFICIENT))
        back_left_thruster_value = int(self.MOTORS_BASE_PWM + (LeftComponent * self.FULL_PWM_RANGE_COEFFICIENT))

        front_right_thruster_value -= _r * self.FULL_ROTATION_COEFFICIENT
        front_left_thruster_value += _r * self.FULL_ROTATION_COEFFICIENT
        back_left_thruster_value += _r * self.FULL_ROTATION_COEFFICIENT
        back_right_thruster_value -= _r * self.FULL_ROTATION_COEFFICIENT

        right_front_thruster_value = int(self.MAXTHRUST *  (front_right_thruster_value - self.PWMNORMAL) + self.PWMNORMAL)
        left_front_thruster_value =  int(self.MAXTHRUST * (front_left_thruster_value - self.PWMNORMAL) + self.PWMNORMAL)
        right_rear_thruster_value =  int(self.MAXTHRUST * (back_right_thruster_value - self.PWMNORMAL) + self.PWMNORMAL)
        left_rear_thruster_value =  int(self.MAXTHRUST * (back_left_thruster_value - self.PWMNORMAL) + self.PWMNORMAL)
        l = [left_front_thruster_value,right_front_thruster_value,right_rear_thruster_value,left_rear_thruster_value]
        return l

class Motion:
    def __init__(self):

        self.emit_signal = None
        # =========== Constants ===========
        self.Forward = 440
        self.Zero_thruster = 290
        self.Brake = 140
        self.Zero_Servo = 450
        self.Servo_min = 300
        self.Servo_max = 600
        self.Joystick_min = -100
        self.Joystick_max = 100
        self.Rotation_Efficiency = 0.23
        self.PWM_Map_Coff = (1 / self.Joystick_max) * (self.Forward - self.Zero_thruster)
        # =========== Motors ==============
        self._horizontalMotors= {}
        self._verticalMotors  = {}
        self._servos =  {}
        self._lights =  {}
        # ======= initialization ===========
        self._stopHorizontalMotors()
        self._stopVerticalMotors()
        self._setCamToNormalPosition()
        self._turnLightOff()
        # ==================================

    def _stopHorizontalMotors(self):
        self._horizontalMotors['Left_Front'] = self.Zero_thruster
        self._horizontalMotors['Right_Front'] = self.Zero_thruster
        self._horizontalMotors['Right_Back'] = self.Zero_thruster
        self._horizontalMotors['Left_Back'] = self.Zero_thruster
    def _stopVerticalMotors(self):
        self._verticalMotors['Vertical_Right'] = self.Zero_thruster
        self._verticalMotors['Vertical_Left'] = self.Zero_thruster
    def _setCamToNormalPosition(self):
        self._servos['Main_Cam'] = self.Zero_Servo
    def _turnLightOff(self):
        self._lights['light'] = 0

    @staticmethod
    def map(value, leftMin, leftMax, rightMin, rightMax):
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        valueScaled = float(value - leftMin) / float(leftSpan)
        return rightMin + (valueScaled * rightSpan)
    def calculateHorizontalMotors_19(self,x,y,r):


        theta = math.atan2(y, x)

        # ============== Skip ============================
        # Check https://ibb.co/ir10AV
        # R = math.hypot(x, y) * (max(abs(math.sin(theta)), abs(math.cos(theta))))
        # ================================================

        # Convert to Circle Coordinates to limit the maximum speed
        # Check https://www.xarg.org/2017/07/how-to-map-a-square-to-a-circle/
        # ================================================
        # The Circle Coordination Equation support [-1,1] range
        X = x / self.Joystick_max
        Y = y / self.Joystick_max

        Xc_2 = X * X * (1 - Y * Y / 2)
        Yc_2 = Y * Y * (1 - X * X / 2)

        R = math.sqrt(Xc_2 + Yc_2) * self.Joystick_max
        # ================================================
        # Axis Rotation
        theta -= math.pi / 4
        # Coefficient to Get the Best Speed of Motors
        # Check https://ibb.co/fYUZ4q   # Check https://ibb.co/dSFqPf
        coff = max(abs(math.sin(theta)), abs(math.cos(theta)))


        # ============ ( C ) is Rotation Efficiency =================
        C = self.map(abs(r),0,self.Joystick_max,0,self.Rotation_Efficiency)
        # C = self.Rotation_Efficiency
        if R ==0:
            C = 1
        # ===========================================================

        R_Cos_Coff = R * math.cos(theta) / coff
        R_Sin_Coff = R * math.sin(theta) / coff

        Motor1 = (1-C) * R_Cos_Coff + C *r
        Motor2 = (1-C) * R_Sin_Coff - C *r
        Motor3 = (1-C) * R_Cos_Coff - C *r
        Motor4 = (1-C) * R_Sin_Coff + C *r

        # Map the Joystick Coordinates to PWM Coordinates

        self._horizontalMotors['Left_Front']  = int(self.Zero_thruster + Motor1 * self.PWM_Map_Coff)
        self._horizontalMotors['Right_Front'] = int(self.Zero_thruster + Motor2 * self.PWM_Map_Coff)
        self._horizontalMotors['Right_Back']  = int(self.Zero_thruster + Motor3 * self.PWM_Map_Coff)
        self._horizontalMotors['Left_Back']  = int(self.Zero_thruster + Motor4 * self.PWM_Map_Coff)

        l = [self._horizontalMotors['Left_Front'] ,self._horizontalMotors['Right_Front'] ,self._horizontalMotors['Right_Back'] ,self._horizontalMotors['Left_Back'] ]
        return l

import socket
tcp = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
tcp.bind(('127.0.0.1',5000))
tcp.listen(1)
conn,addr = tcp.accept()
print('connected ya ray2')
E1=E()
E2=Motion()
while True:
    msg = conn.recv(1024).decode()
    qt_string = Split_to_Dict(msg)
    x= map(qt_string['x'],-32768,32768,-100,100)
    y= map(qt_string['y'],-32768,32768,-100,100)
    r= map(qt_string['r'],-32768,32768,-100,100)

    e1 = E1.calculateHorizontalMotors_17(x,y,r)
    e2 = E2.calculateHorizontalMotors_19(x,y,r)
    print(e1, '   ', e2)


