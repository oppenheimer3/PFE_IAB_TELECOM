
from dronekit import connect, VehicleMode,LocationGlobalRelative, Command, LocationGlobal
from QMapWidget import haversine
from pymavlink import mavutil
from PyQt5.QtCore import  QThread,pyqtSignal
import time

def first_connect(str):
    connect(str)


class dronecontrol(QThread):
    con_signal=pyqtSignal()
    def __init__(self,conn_string):
        super().__init__()
        self.threadactive = True
        self.conn_string=conn_string
        self.drone=None
    def run(self):
        if not self.drone:
            try:
                self.drone=connect(self.conn_string)
                self.cmds=self.drone.commands
                self.cmds.download()
                self.con_signal.emit()
            except:pass

    def close(self):
        self.drone.close()


    def stop(self):
        self.threadactive = False
        self.wait()

    def state(self):
        lat=self.drone.location.global_frame.lat
        lon=self.drone.location.global_frame.lon
        alt=self.drone.location.global_relative_frame.alt
        yaw=self.drone.attitude.yaw
        speed=self.drone.airspeed
        battery=self.drone.battery.level
        return {'lat':lat,'lon':lon,'alt':alt ,'yaw':yaw , 'speed':speed , 'battery':battery}

    def set_mode(self,mode):
        self.drone.mode=VehicleMode(mode)

    def set_home(self,cords):
        self.drone.home_location=LocationGlobal(cords['lat'],cords['lon'],cords['alt'])


    def is_armable(self):
        return self.drone.is_armable

    def arm_throttle(self):
        self.drone.armed=True

    def disarm(self):
        self.drone.armed=False
    
    def simple_takeoff(self,alt):
        self.drone.simple_takeoff(alt)
    

    def pymavlin_mission(self,cords):
        self.cmds.clear()
        self.cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
        for i in range(len(cords)):
            cord=cords[i]
            self.cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, cord['lat'], cord['lng'], 10))
        
        self.cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0 , 0, 0))


        self.cmds.upload()
        self.drone.commands.next=0


class simple_mission(QThread):
    mission_signal=pyqtSignal(str)
    def __init__(self,drone):
        super().__init__()
        self.drone=drone
    def run(self):
        def arm_and_takeoff(aTargetAltitude):


            self.mission_signal.emit("Basic pre-arm checks")
            # Don't let the user try to arm until autopilot is ready
            while not self.drone.is_armable():
                self.mission_signal.emit(" Waiting for drone to initialise...")
                time.sleep(1)

                
            self.mission_signal.emit("Arming motors")
            # Copter should arm in GUIDED mode
            self.drone.set_mode("GUIDED")
            self.drone.arm_throttle()

            while not self.drone.drone.armed:      
                self.mission_signal.emit(" Waiting for arming...")
                time.sleep(1)

            self.mission_signal.emit("Taking off!")
            self.drone.simple_takeoff(aTargetAltitude) # Take off to target altitude

            # Wait until the self.drone reaches a safe height before processing the goto (otherwise the command 
            #  after self.drone.simple_takeoff will execute immediately).
            while True:
                if self.drone.drone.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
                    self.drone.set_mode("AUTO")
                    self.mission_signal.emit("In mission")

                    break
                time.sleep(1)
        arm_and_takeoff(10)
