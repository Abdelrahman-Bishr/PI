import threading
import ms5837
import time


class Sensor_Interrupter :
    def __init__(self,time):
        self.pressure = 0
        self.pwm = 305
        self.sensor = ms5837.MS5837_30BA()
        self.time = time

        if not self.sensor.init():
            exit(1)
        if not self.sensor.read():
            exit(1)

        print("Pressure: ", self.sensor.pressure(ms5837.UNITS_atm))
        print("Temperature: ", self.sensor.temperature(ms5837.UNITS_Centigrade))

        self.freshwaterDepth = self.sensor.depth()  # default is freshwater
        self.sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)

        time.sleep(5)

    def SIGNAL_Referance(self,Observer_Pattern_Signal):
        self.emit_signal=Observer_Pattern_Signal

    def interrupt(self,update,name):
        self.update_pressure()
        update(name,self.pressure,self.pwm)
        threading.Timer(self.time,self.interrupt,[update,name]).start()

    def update_pressure(self):

        if self.sensor.read():
            self.pressure = self.sensor.pressure()
            print("Pressure: ", sensor.pressure(), "Temperature: ", sensor.temperature())
        else:
                print("feh moshkela fel sensor")

    def update_pwm(self,event,z:int):
        if event == "SENSOR":
            self.pwm = z
            print(self.pwm)
