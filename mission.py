import socket
from pymavlink import mavutil
import threading
import time
import math
import ast
import cv2
import json
import numpy as np

master = mavutil.mavlink_connection('tcp:192.168.2.1:5760')
master.wait_heartbeat()
master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)

threadactive=True


def capture(last_waypoint):
    gps_info={}
    # Set up the camera object
    cap1 = cv2.VideoCapture(2)
    cap2 = cv2.VideoCapture(0)
    # Define the path where the images will be saved
    save_path1 = "mission/images/ndvi/"
    save_path2 = "mission/images/rgb/"
    prev_time = time.time()
    
    while True:
        # Capture a frame from the camera
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
        # Check if the frame was captured successfully
        if not ret1:
            print("Failed to capture frame 1")
            continue
        if not ret2:
            print("Failed to capture frame 2")
        # Split the frame into individual color channels
        red, green, nir = cv2.split(frame1)

        # Calculate the (nir - red) / (nir + red) ratio with handling division by zero
        with np.errstate(divide='ignore', invalid='ignore'):
            ratio = np.divide((nir.astype(np.float32) - red.astype(np.float32)), (nir.astype(np.float32) + red.astype(np.float32)))
            ratio[np.isnan(ratio)] = 0  # Replace NaN (result of 0/0) with 0

        # Normalize the ratio to the range [0, 255]
        ratio_normalized = cv2.normalize(ratio, None, 0, 255, cv2.NORM_MINMAX)

        # Apply a color map to enhance visualization
        ratio_colormap = cv2.applyColorMap(ratio_normalized.astype(np.uint8), cv2.COLORMAP_JET)

        curr_time = time.time()
        time_diff = curr_time - prev_time

        if time_diff >= 1:
            msg = master.recv_match(type="GLOBAL_POSITION_INT",blocking=True).to_dict()
            gps_info[f"image_{time.time()}"]={'lon':msg['lon']*1e-7,
                                      'lat':msg['lat']*1e-7,
                                      'alt':msg['relative_alt']/1000}
            prev_time = curr_time
            filename1 = f"{save_path1}image_{time.time()}.jpg"
            filename2 = f"{save_path2}image_{time.time()}.jpg"
            cv2.imwrite(filename1, ratio_colormap)
            cv2.imwrite(filename2, frame2)


        
        msg0 = master.recv_match(type='MISSION_ITEM_REACHED')
        if msg0:
            msg0=msg0.to_dict()
            print(f"Reached waypoint n:{msg0['seq']}")
            if msg0['seq']==last_waypoint-1:
                print('closing the camera')
                break
        
        
    cap1.release()
    cap2.release()
    with open("mission/gps_info.json", "w") as f:
        json.dump(gps_info, f)





def drone_heart_beat():
    while threadactive:
        msg = master.recv_match(blocking=True).to_dict()
        if not msg:
            continue
        # print(msg)
        time.sleep(3)
drone_thread=threading.Thread(target=drone_heart_beat)
drone_thread.start()

def ack_message(kw):
    msg=master.recv_match(type=kw, blocking=True)
    print(msg)
    return
def change_mode(mode):
    # Get mode ID
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

def load_mission(cords):
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    ack_message('MISSION_ACK')
    master.mav.mission_count_send(master.target_system, master.target_component,len(cords)+1,0)
    ack_message('MISSION_REQUEST')

    for i,cord in enumerate(cords):
        print(f'sending waypoint {i}')
        master.mav.mission_item_int_send(master.target_system, master.target_component,i,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,1,0,2,20,math.nan,int(cord['lat'] * 1e7),int(cord['lng'] * 1e7),4,mavutil.mavlink.MAV_MISSION_TYPE_MISSION) 
        ack_message('MISSION_REQUEST')
    print('sending RTL command')
    master.mav.mission_item_int_send(master.target_system, master.target_component, len(cords), mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0 , 0, 0)          
    ack_message('MISSION_ACK')
    connection.send('Mission loaded'.encode())
def start_mission(a):
    print('seting mode GUIDED')
    change_mode('GUIDED')
    master.arducopter_arm()
    master.mav.command_long_send(
    master.target_system, 
    master.target_component, 
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,0,0,0,0,0,0,4)
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT",blocking=True).to_dict()
        print(msg['relative_alt']/1000)
        
        if msg['relative_alt']/1000 > 3.9:
            break
    print('seting mode AUTO')
    change_mode('AUTO')
    while True:
        msg = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True).to_dict()
        if msg:
            # the sequence number of the first waypoint is in the "param1" field
            print(f"Reached first waypoint")
            print('opening camera')
            break
        time.sleep(0.1)
    capture(a)


# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host='192.168.2.2'
# Bind the socket to a specific address and port
server_address = (host, 8080)
print(f'starting up on {server_address[0]}:{server_address[1]}')
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)


while True:
    # Wait for a connection
    print('waiting for a connection...')
    connection, client_address = sock.accept()

    print(f'new connection from {client_address[0]}:{client_address[1]}' )
    
    print('waiting for mission coordinates...')
    
    cords=connection.recv(1084576).decode()
    cords= ast.literal_eval(cords)
    print('got mission coordinates')
    print('loading mission to drone')
    threadactive=False
    drone_thread.join()
    drone_thread=threading.Thread(target=load_mission,args=[cords])
    drone_thread.start()
    drone_thread.join()
    print('waiting for mission start command...')

    message=connection.recv(1024).decode()

    if message=='start':
        print('closing connection and starting mission')
        connection.close()
        sock.close()
        drone_thread=threading.Thread(target=start_mission,args=[len(cords)])
        drone_thread.start()
        drone_thread.join()
        
        break
    else:
        print(message) 
        connection.close()
        sock.close
        break










    






