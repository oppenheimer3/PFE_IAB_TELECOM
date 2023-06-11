from pymavlink import mavutil
from PyQt5.QtCore import  QObject,pyqtSignal,pyqtSlot
import time
class drone(QObject):
    connected=pyqtSignal(dict)
    waypoint_signal=pyqtSignal(int)
    heart_beat=pyqtSignal(dict)
    def __init__(self,con_str):
        super().__init__()
        self.threadactive = True
        self.master=None
        self.con_str=con_str

    @pyqtSlot()
    def connect(self):
        try :
            self.master = mavutil.mavlink_connection(self.con_str,baud=57600)
        except Exception as e:
            print(e)

        if not self.master:
            return                 

        self.master.wait_heartbeat()
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
        drone_mode = mavutil.mode_string_v10(msg)

        self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        msg_pos0 = self.master.recv_match(type="GLOBAL_POSITION_INT",blocking=True).to_dict()
        msg_att0=self.master.recv_match(type="ATTITUDE",blocking=True).to_dict()
        self.connected.emit({'lon':msg_pos0['lon']*1e-7,
                                      'lat':msg_pos0['lat']*1e-7,
                                      'yaw':msg_att0['yaw'],
                                      'mode':drone_mode})
    def request_hb(self): 

        msg_pos = self.master.recv_match(type="GLOBAL_POSITION_INT",blocking=True).to_dict()
        msg_att=self.master.recv_match(type="ATTITUDE",blocking=True).to_dict()
        msg_batterry = self.master.recv_match(type="BATTERY_STATUS",blocking=True).to_dict()
        msg_speed=self.master.recv_match(type="VFR_HUD",blocking=True).to_dict()
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
        drone_mode = mavutil.mode_string_v10(msg)
        return {'lon':msg_pos['lon']*1e-7,
                                    'lat':msg_pos['lat']*1e-7,
                                    'yaw':msg_att['yaw'],
                                    'alt':msg_pos['relative_alt']/1000,
                                    'battery':msg_batterry['battery_remaining'],
                                    'speed': msg_speed['airspeed'],
                                    'mode':drone_mode}



    @pyqtSlot()
    def get_heart_beat(self):
        # Continuously read messages from the MAVLink source
        while self.threadactive == True:
            try:
                msg=self.request_hb()
                if not msg['lon']:
                    continue
                self.heart_beat.emit(msg)
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(e)
    def close(self):
        self.threadactive= False
        time.sleep(0.1)
        self.master.close()


        










        




