import time
import board
import adafruit_adxl34x

class accell():

    def __init__(self):
        self.i2c = board.I2C()
        self.break_loop = False

    def init_connection(self,thersh_hold=16):
        self.accelerometer = adafruit_adxl34x.ADXL343(self.i2c)
        self.accelerometer.enable_motion_detection(threshold=thersh_hold)

    def start_loop_detections(self):
        while True:
            print("{} {} {}".format(*accelerometer.acceleration))
            if(self.break_loop):
                break
            print("Motion detected: {}".format(accelerometer.events["motion"]))
            time.sleep(0.5)

if __name__ == "__main__":
    ac = accell()
    ac.init_connection()
    ac.start_loop_detections()
    time.sleep(10)
    ac.break_loop = True

