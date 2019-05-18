from picamera.array import PiRGBArray 
from picamera import PiCamera 
import cv2 
import smbus
import math
import time  
import pyfirmata 
import datetime 
import numpy as np 
#from picamera.array import PiRGBArray 
#from picamera import Picamera 
#import cv2 
#if __name__== "__main__": 
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
AngleDegX  = 0 
AngleDegy = 0 
AngleDegz = 0 
# Camera picam 
camera = PiCamera() 
camera.resolution = (640,480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
fourcc = cv2.cv.CV_FOURCC(*'XVID') # cv.videowriter   
times = datetime.datetime.now()
videoOut = cv2.VideoWriter("UUV"+str(times)+".avi",fourcc,20.0,(640,480)) # Videowriter 
 
# allow the camera to warmup
time.sleep(0.1)
try:
  hardware = pyfirmata.ArduinoMega("/dev/ttyUSB0")
except: 
  print("connecting the backup protocol")
try:
   hardware = pyfirmata.ArduinoMega("/dev/ttyUSB1")
except:
  print("All hardware not connected")
XLservo = hardware.get_pin('d:8:s')
XRservo = hardware.get_pin('d:9:s') 
YUservo = hardware.get_pin('d:10:s')
YBservo = hardware.get_pin('d:11:s')

def read_byte(reg):
    return bus.read_byte_data(address, reg)
def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value
def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)
bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
address = 0x68       # via i2cdetect

# Aktivieren, um das Modul ansprechen zu koennen
bus.write_byte_data(address, power_mgmt_1, 0)
 
print("Gyroscope")
print("--------")
gyroskop_xout = read_word_2c(0x43)
gyroskop_yout = read_word_2c(0x45) 
gyroskop_zout = read_word_2c(0x47)
def Servoanglecontrol(AngleY,Anglez): 
     XLservo.write(AngleY)
     XRservo.write(180-AngleY)  
     YUservo.write(Anglez)
     YBservo.write(180-Anglez)      
Blackdata = open("UUVreportData.txt","w+")
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        
           #Video start record
        videoOut.write(image)
           # show the frame
        cv2.imshow("UUV online ...", image)
        key = cv2.waitKey(1) & 0xFF
 
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        
        key = cv2.waitKey(1)
        print("UUV Gyroscope") 
        print("---------------------")
 
        beschleunigung_xout = read_word_2c(0x3b)
        beschleunigung_yout = read_word_2c(0x3d)
        beschleunigung_zout = read_word_2c(0x3f)

        beschleunigung_xout_skaliert = beschleunigung_xout / 16384.0
        beschleunigung_yout_skaliert = beschleunigung_yout / 16384.0
        beschleunigung_zout_skaliert = beschleunigung_zout / 16384.0
        AngleDegX = math.degrees(beschleunigung_xout_skaliert)
        AngleDegY = math.degrees(beschleunigung_yout_skaliert)
        AngleDegZ = math.degrees(beschleunigung_zout_skaliert)
        print("AngleDegX",(AngleDegX))
        print("AngleDegY",(AngleDegY))
        print("AngleDegZ",(AngleDegZ))
        print("Acceleration_xout",(beschleunigung_xout_skaliert))
        print("Acceleration_yout",(beschleunigung_yout_skaliert))
        print("Acceleration_zout",(beschleunigung_zout_skaliert))
        #GyroSense = str(AngleDegX) + "," + str(AngleDegY) + "," + str(AngleDegZ)
        Blackdata.write("\n"+str(times)+"Gyroscope data(x,y,z)"+"," +str(AngleDegX) +","+str(AngleDegY)+","+str(AngleDegZ) +","+"Accelerometer"+ str(beschleunigung_xout_skaliert)+ ","+ str(beschleunigung_yout_skaliert) +","+ str(beschleunigung_zout_skaliert))
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(image,"Gyroscope data(x,y,z)",(150,50),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),1,-1)
        Servoanglecontrol(abs(AngleDegY),abs(AngleDegZ))
videoOut.release() 



      
