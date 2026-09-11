import gpsd
import time

class gps():
    def __init__(self, myhost="host.docker.internal", myport=2947):
        gpsd.connect(host=myhost, port=myport)
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.hspeed = 0.0
        self.sats = 0.0
        self.break_out = False

    def get_GPS_data(self):
        while True:
            packet = gpsd.get_current()
            print(f"Lat: {packet.lat}")
            print(f"Lon: {packet.lon}")
            print(f"Alt: {packet.alt}")
            print(f"Speed: {packet.hspeed}")
            print(f"Satellites: {packet.sats}")
            print("-" * 40)
            self.lat = packet.lat
            self.lon = packet.lon
            self.alt = packet.alt
            self.hspeed = packet.hspeed
            self.sats = packet.sats
            if(self.break_out)
                break
            time.sleep(1)
if __name__ == "__main__":
    gp = gps()
    ac.init_connection()
    ac.start_loop_detections()
    time.sleep(10)
    ac.break_loop = True
