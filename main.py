import sys
import QMapWidget
from PyQt5.QtCore import QTimer, pyqtSlot,QEventLoop ,pyqtSignal,QObject,QThread
from PyQt5.QtWidgets import QFileDialog 
import math
import droneControl
import main_widget 
from PyQt5 import  QtWidgets 
import socket
from drone_connection import drone
import ssh



class signal(QObject):
    initialize_signal=pyqtSignal()
    tile_signal=pyqtSignal(dict)
    con_signal=pyqtSignal()
    start_mission_signal=pyqtSignal()
    load_mission_signal=pyqtSignal(list)
    request_heartbeat_signal=pyqtSignal()


class MyApp(main_widget.Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.map=QMapWidget.MapWidget()
        self.timer = QTimer()
        self.timer.timeout.connect(self.track)
        self.mission_timer = QTimer()  
        self.loop = QEventLoop()   
        self.loop2 = QEventLoop()
        self.signal=signal()
        self.uploaded_leaf_image=None
        self.drone=None
        self.ee_initialized=False   
        self.map.page.console_signal.connect(self.enable_buttons)
        self.num_layers=0
        self.connection_thread=QThread()
        self.connection_thread.start()


    def setupUi(self):
        super().setupUi(MainWindow)
        self.verticalLayout.addWidget(self.map)  
        self.display_label.setText('Display') 
        self.connect_button.clicked.connect(self.connect)
        self.simulation_button.clicked.connect(self.run_simulation)
        self.mission_button.clicked.connect(self.load_mission)
        self.mission_button.setEnabled(False)
        self.arm_button.clicked.connect(self.arm_throttle)
        self.arm_button.setEnabled(False)
        self.plan_button.clicked.connect(self.plan_mission)
        self.plan_button.setEnabled(False)
        self.drone_image_button.clicked.connect(self.add_drone_image)
        self.connect_pi.clicked.connect(self.socket_connection)
        self.download.clicked.connect(ssh.download)
        


    def set_display(self,msg):
        self.display_label.setText(msg)




    def enable_buttons(self):
        if not self.plan_button.isEnabled():
            self.plan_button.setEnabled(True)



    def set_text(self,text):
        loop = QEventLoop()
        QTimer.singleShot(1, loop.quit)
        self.display_label.setText(text)
        loop.exec_() 
 



    def run_simulation(self):
        import sitl
        sitl.run_simulation()
        droneControl.first_connect('tcp:127.0.0.1:5760')
        self.simulation_button.setEnabled(False)

    def socket_connection(self):
        ssh.listen()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = ('192.168.2.2', 8080)
        try:
            self.sock.connect(server_address)
        except:
            self.display_label.setText('Failed')
            return
        self.display_label.setText('connected to raspberrypi')
        self.connect_pi.setEnabled(False)

    def connected(self, hb):
        self.map.add_marker([hb['lat'],hb['lon']],'./static/399308.png',hb['yaw'])
        self.map.center([hb['lat'],hb['lon']])
        self.connect_button.setText('Disconnect')
        self.connect_button.clicked.disconnect()
        self.connect_button.clicked.connect(self.disconnect)
        self.signal.request_heartbeat_signal.emit()


    def connect(self):
        self.drone=drone(self.connection_string.toPlainText())
        self.drone.moveToThread(self.connection_thread)
        self.drone.connected.connect(self.connected)
        self.drone.heart_beat.connect(self.on_connection)
        self.signal.request_heartbeat_signal.connect(self.drone.get_heart_beat)
        self.signal.con_signal.connect(self.drone.connect)
        self.signal.con_signal.emit()



    def on_connection(self,hb):

        self.lcd_altitude_3.display(str('%.2f' % hb['alt']))
        self.lcd_battery_3.display(str(hb['battery']))
        self.lcd_speed_3.display(str('%.2f' % hb['speed']))
        self.mode_label.setText(hb['mode'])
        self.map.update_marker([hb['lat'],hb['lon']],'./static/399308.png',hb['yaw']*(180/math.pi))





    def disconnect(self):
        self.drone.close()
        self.connect_button.setText('Connect')
        self.connect_button.clicked.connect(self.connect)






    def track(self):
        drone_data=self.drone.state()
        self.map.update_marker([drone_data['lat'],drone_data['lon']],'./static/399308.png',drone_data['yaw']*(180/math.pi))
        self.lcd_altitude_3.display(drone_data['alt'])
        self.lcd_speed_3.display(drone_data['speed'] )
        self.lcd_battery_3.display(drone_data['battery'] )
        self.mode_label.setText(drone_data['mode'])


    
    def arm_throttle(self):
        self.sock.send('start'.encode())
        self.arm_button.setEnabled(False)
        self.display_label.setText('taking off!')

    def plan_mission(self):
        self.map.polyline_in_polygon()
        self.plan_button.setEnabled(False)
        # if self.connect_pi.isEnabled==False:
        self.mission_button.setEnabled(True)
           

    def load_mission(self):
        cords=self.map.get_polyline_coordinates()
        self.sock.send(str(cords).encode())
        self.arm_button.setEnabled(True)
        self.mission_button.setEnabled(False)
        self.display_label.setText(self.sock.recv(1024).decode())




    def add_drone_image(self):
        import rasterio as rio

        tif_file = QFileDialog.getOpenFileName(MainWindow, 'Open file', 
         'c:\\',"Image files (*.tif )")

        src = rio.open(tif_file[0])


        from PIL import Image

        # Open the TIFF file using the Pillow library
        with Image.open(tif_file[0]) as img:

            # Convert the image to RGB format if it's not already in that format
            if img.mode != 'RGB':
                img = img.convert('RGB')

            # Save the image in PNG format
            img.save('file.png', format='PNG')

        import pyproj
        transformer = pyproj.Transformer.from_crs(src.crs, 'EPSG:4326', always_xy=True)
        bounds=transformer.transform_bounds(src.bounds[0],src.bounds[1],src.bounds[2],src.bounds[3])

        bounds = [[bounds[1], bounds[0]], [bounds[3], bounds[2]]]
        print(bounds)
        self.map.image_overlay('"file.png"',bounds)
   

   
        




if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = MyApp()
    ui.setupUi()
    MainWindow.show()
    sys.exit(app.exec_())



