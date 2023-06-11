import piexif
from PIL import Image
import json
import os

with open('../mission/gps_info.json', 'r') as file:
    gps_logs = json.load(file)

def get_gps_data(image_path):
    image = Image.open(image_path)
    try:
        exif_data = piexif.load(image.info["exif"])
    except:
        print('no exif data')
        return
    if "GPS" in exif_data:
        gps_data = exif_data["GPS"]
        # Access individual GPS tags like "GPSLatitude", "GPSLongitude", etc.
        latitude = gps_data.get(piexif.GPSIFD.GPSLatitude)
        longitude = gps_data.get(piexif.GPSIFD.GPSLongitude)
        print(gps_data)
        # Process or use the GPS data as needed
    else:
        print("No GPS data found in the image.")


def add_gps_data(image_path,gps_data=None):
    gps = {
        
            piexif.GPSIFD.GPSLatitudeRef: get_latitude_direction(gps_data['lat']),  # Latitude reference (North)
            piexif.GPSIFD.GPSLatitude: convert_to_dms(gps_data['lat']),
            piexif.GPSIFD.GPSLongitudeRef: get_longitude_direction(gps_data['lon']),  # Longitude reference (East)
            piexif.GPSIFD.GPSLongitude: convert_to_dms(gps_data['lon']),
            piexif.GPSIFD.GPSAltitudeRef: 0,  # Altitude reference (above sea level)
            piexif.GPSIFD.GPSAltitude:(int(gps_data['alt']*100), 100)  # Altitude in meters
        }
    exif_data = piexif.load(image_path)
    exif_data.update({"GPS":gps})
    exif_bytes = piexif.dump(exif_data)
    piexif.insert(exif_bytes, image_path,image_path)

def convert_to_dms(latitude):
    abs_latitude = abs(latitude)
    degrees = int(abs_latitude)
    decimal_minutes = (abs_latitude - degrees) * 60
    minutes = int(decimal_minutes)
    seconds = int((decimal_minutes - minutes) * 60 * 10000)

    # Format the DMS components
    dms = ((degrees,1), (minutes,1), (seconds,10000))

    # Return the formatted DMS as a tuple
    return dms

def get_latitude_direction(latitude):
    if latitude >= 0:
        return 'N'
    else:
        return 'S'
def get_longitude_direction(longitude):
    if longitude >= 0:
        return 'E'
    else:
        return 'W'


keys=list(gps_logs.keys())
path = 'C:\\Users\\PC\Desktop\\dataset\\project\\images\\'
lst_dir = os.listdir(path)
for i , image in enumerate(lst_dir):
    image_name=keys[i]
    add_gps_data(os.path.join(path,image),gps_logs[image_name])
