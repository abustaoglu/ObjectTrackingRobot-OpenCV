from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
import imutils

#cam def

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
cap = PiRGBArray(camera, size=(640, 480))


#motor pins
GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM)

RF1 = 22
RF2 = 27

RB1 = 14
RB2 = 4

LF1 = 24
LF2 = 23

LB1 = 17
LB2 = 18

buz = 10

#gpio setup and lowstart
GPIO.setup(RF1, GPIO.OUT)
GPIO.setup(RF2, GPIO.OUT)
GPIO.setup(RB1, GPIO.OUT)
GPIO.setup(RB2, GPIO.OUT)
GPIO.setup(LF1, GPIO.OUT)
GPIO.setup(LF2, GPIO.OUT)
GPIO.setup(LB1, GPIO.OUT)
GPIO.setup(LB2, GPIO.OUT)

GPIO.setup(buz, GPIO.OUT)


GPIO.output(RF1, GPIO.LOW)
GPIO.output(RF2, GPIO.LOW)

GPIO.output(RB1, GPIO.LOW)
GPIO.output(RB2, GPIO.LOW)
    
GPIO.output(LF1, GPIO.LOW)
GPIO.output(LF2, GPIO.LOW)
    
GPIO.output(LB1, GPIO.LOW)
GPIO.output(LB2, GPIO.LOW)

GPIO.output(buz, GPIO.LOW)


def rf(): #oldu

    GPIO.output(RF1, GPIO.HIGH)
    GPIO.output(RF2, GPIO.LOW)
    
def rb(): # oldu

    GPIO.output(RB1, GPIO.HIGH)
    GPIO.output(RB2, GPIO.LOW)

def lf(): #oldu

    GPIO.output(LF1, GPIO.HIGH)
    GPIO.output(LF2, GPIO.LOW)
    
def lb(): #oldu

    GPIO.output(LB1, GPIO.HIGH)
    GPIO.output(LB2, GPIO.LOW)

def forward():
    

    GPIO.output(RF1, GPIO.HIGH)
    GPIO.output(RF2, GPIO.LOW)

    GPIO.output(RB1, GPIO.HIGH)
    GPIO.output(RB2, GPIO.LOW)


    GPIO.output(LF1, GPIO.HIGH)
    GPIO.output(LF2, GPIO.LOW)
    

    GPIO.output(LB1, GPIO.HIGH)
    GPIO.output(LB2, GPIO.LOW)



def reverse():
    

    GPIO.output(RF1, GPIO.LOW)
    GPIO.output(RF2, GPIO.HIGH)

    GPIO.output(RB1, GPIO.LOW)
    GPIO.output(RB2, GPIO.HIGH)


    GPIO.output(LF1, GPIO.LOW)
    GPIO.output(LF2, GPIO.HIGH)
    

    GPIO.output(LB1, GPIO.LOW)
    GPIO.output(LB2, GPIO.HIGH)    
    
    
def rightturn():
    
    
    GPIO.output(RF1, GPIO.LOW)
    GPIO.output(RF2, GPIO.LOW)

    GPIO.output(RB1, GPIO.LOW)
    GPIO.output(RB2, GPIO.LOW)


    GPIO.output(LF1, GPIO.HIGH)
    GPIO.output(LF2, GPIO.LOW)
    

    GPIO.output(LB1, GPIO.HIGH)
    GPIO.output(LB2, GPIO.LOW)
    
def leftturn():
    
    
    GPIO.output(RF1, GPIO.HIGH)
    GPIO.output(RF2, GPIO.LOW)

    GPIO.output(RB1, GPIO.HIGH)
    GPIO.output(RB2, GPIO.LOW)


    GPIO.output(LF1, GPIO.LOW)
    GPIO.output(LF2, GPIO.LOW)
    

    GPIO.output(LB1, GPIO.LOW)
    GPIO.output(LB2, GPIO.LOW)
    
def stop():
    
    
    GPIO.output(RF1, GPIO.LOW)
    GPIO.output(RF2, GPIO.LOW)

    GPIO.output(RB1, GPIO.LOW)
    GPIO.output(RB2, GPIO.LOW)


    GPIO.output(LF1, GPIO.LOW)
    GPIO.output(LF2, GPIO.LOW)
    

    GPIO.output(LB1, GPIO.LOW)
    GPIO.output(LB2, GPIO.LOW)
    
def buzzer():
    GPIO.output(buz, GPIO.HIGH)

    time.sleep(0.1)
    
    GPIO.output(buz, GPIO.LOW)
    
    time.sleep(0.5)
    
    
#sensor part

TRIG = 21      
ECHO = 20


GPIO.setup(TRIG,GPIO.OUT) 
GPIO.setup(ECHO,GPIO.IN) 

GPIO.output(TRIG, False)

def sonar():
      
      start=0
      stop=0
      # SetPins
      GPIO.setup(TRIG,GPIO.OUT)  
      GPIO.setup(ECHO,GPIO.IN)      

    
      GPIO.output(TRIG, True)
      
      time.sleep(0.00001) #10uf call
      
      GPIO.output(TRIG, False)
     
      begin = time.time()
      
      while GPIO.input(ECHO)==0 and time.time()<begin+0.05:
      
          start = time.time()
     
      while GPIO.input(ECHO)==1 and time.time()<begin+0.1:
            stop = time.time()
     
  
      elapsed = stop-start

      distance = elapsed * 17500
     
      
      return distance

YellowLower = (0, 125, 88)
YellowUpper = (179, 255, 255)
kernel = np.ones((3,3), np.uint8); # for mask
font = cv2.FONT_HERSHEY_SIMPLEX 
#get frame after 2 seconds
#time.sleep(15)


for frame in camera.capture_continuous(cap, format="bgr", use_video_port=True):

    frame = frame.array

    frame = cv2.flip(frame, -1) # flip 180 degree
    
    frame = cv2.GaussianBlur(frame, (5,5), 0) #smoothing
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(hsv, YellowLower, YellowUpper)
    
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
   
    cnts = imutils.grab_contours(cnts) # denoising
    
    frame_x, frame_y, _ = frame.shape 
    
    min_area = 0; # min rect. shape w*h
    
    x_point = 0; # frame center
    
    y_point = 0;   #frame center 
    
    #if camera find the ball
    
    if len(cnts) > 0:
    
        c = max(cnts, key=cv2.contourArea) #maximum contour
        
        (x, y, w, h) = cv2.boundingRect(c) 
        #x and y starting point, w and h height and width ratio
        
        cv2.rectangle(frame, (x,y),(x+w,y+h), (0,255,0), 3) # min rectangular drawing
        
        frame = cv2.putText(frame, 'Ball', (x+w-20,y+h+30), font, 1, (255,0,255), 1, cv2.LINE_AA) 
                   
        min_area = w*h #rectangular area
        
        x_point = x + (w/2) # obje center
        
        y_point = y + (h/2)
        
        dis = sonar() #distance
        
        if ((min_area > 700 ) and (min_area < 25000)) and (dis > 15):
            
            
            if x_point > ((frame_y/2) + (frame_y/3)):     
        
                rightturn()
                
                time.sleep(0.5)
                
                stop()
                
                print("turning right------")
                
                
            elif x_point < ((frame_y/2) - (frame_y/3)):
               
                
                leftturn()
                
                time.sleep(0.5)
                
                stop()

                
                print("Turning left----------------")
                
            else:
                
                #time.sleep(0.5)
                
                forward()
                
                #time.sleep(2)

                
                print('go ahead--','dis is = ',dis,'area = ', min_area)
                
        
        elif dis < 20:
            
            #time.sleep(0.5)
            
            stop()
            
            buzzer()
            
            print('target so close', 'distance is =  ',dis)
        
        elif (min_area > 300) and (min_area < 700)  :
            
            forward()
            
            time.sleep(2)
                    
            stop()
      
            time.sleep(1)
            
            print('too far, close it = ',dis, 'area = ', min_area)

    else:
        
        rightturn()
        
        buzzer()
        
        
        #time.sleep(0.7)
        
       # stop()
      
        #time.sleep(1)

        print('there is no target, searching')
    
    cv2.imshow("Frame", frame)
    
    #cv2.imshow('masked ball', mask)
    
    # release cap
    
    cap.truncate(0)
    

    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        
        stop()
        
        GPIO.cleanup()
        
        break

cv2.destroyAllWindows()