from picamera.array import PiRGBArray
from picamera import PiCamera
import paho.mqtt.publish as publish
import RPi.GPIO as GPIO
import time
import cv2
import os
import pigpio

#import Adafruit_ADS1x15
import paho.mqtt.client as mqtt


port = 1883 # default port match this with ur mqqt port
Server_ip = "192.168.1.16" #your ip address

Publish_Topic = "/sensor"  
Subscribe_St_Topic = "/steering" 
Subscribe_Sp_Topic = "/speed"
Subscribe_Rc_Topic = "/record"


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
p_servo = pigpio.pi()
p_servo_u = pigpio.pi()
p_servo.set_mode(12, pigpio.OUTPUT) #steering servo
p_servo_u.set_mode(18, pigpio.OUTPUT) #ultrasonic servo
p_servo.set_PWM_frequency(12, 50)
p_servo_u.set_PWM_frequency(18, 50)

GPIO.setup(13,GPIO.OUT)    #motor pin
GPIO.setup(19, GPIO.OUT)
p1 = GPIO.PWM(13, 50)
p2 = GPIO.PWM(19 ,50)
GPIO.setup(20,GPIO.OUT)
GPIO.setup(21,GPIO.OUT)
GPIO.setup(2,GPIO.OUT)
GPIO.setup(3,GPIO.OUT)
p1.start(0)
p2.start(0)

def on_steering(pwm): #Turn on steering motor
       p_servo.set_servo_pulsewidth(12, pwm)
       x = scaledata(pwm)
       if(x <= 1150):
            x = 1150
       elif(x >= 2000):
            x = 2000
       p_servo_u.set_servo_pulsewidth(18, x)
             
def on_drive(pwm): #ur8 on speed motor
       mult = 0.55
       #track1 .45 - .5 track2 .56-62
       if(pwm > 5):
           GPIO.output(20,GPIO.HIGH)#IN1
           GPIO.output(21,GPIO.LOW)#IN2
           GPIO.output(2,GPIO.HIGH)#IN1
           GPIO.output(3,GPIO.LOW)#IN2
       elif(5 <= pwm <= -5):
           GPIO.output(20,GPIO.HIGH)#IN1
           GPIO.output(21,GPIO.HIGH)#IN2
           GPIO.output(2,GPIO.HIGH)#IN1
           GPIO.output(3,GPIO.HIGH)#IN2
       elif(pwm < -5):
           GPIO.output(20,GPIO.LOW)#IN1
           GPIO.output(21,GPIO.HIGH)#IN2
           GPIO.output(2,GPIO.LOW)#IN1
           GPIO.output(3,GPIO.HIGH )#IN2
       pwm = abs(round(pwm * mult))
       p2.ChangeDutyCycle(pwm) #motor one percentage speed
       p1.ChangeDutyCycle(pwm)
    
def myround(x, base=50):    #rounding for servo motor
    return base * round(x/base)

def scaledata(x):  #reverse the values of steering servo so it can be compaitaible together
  y = (x - 1100)/1000
  y = float("{0:.2f}".format(y))
  y = abs(y - 1)
  y = (y * 1000) + 1100
  y = myround(y)
  return y


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    client.subscribe(Subscribe_Rc_Topic)
    client.subscribe(Subscribe_Sp_Topic)
    client.subscribe(Subscribe_St_Topic)
# The callback for when a PUBLISH message is received from the server.


rc_f = 0
i = 31
steering_a = 0
def on_message(client, userdata, msg):
    global rc_f
    global i
    global steering_a
    #print(msg.topic+" "+int(msg.payload))
    if(msg.topic == Subscribe_St_Topic):
        #print("steering " ,int(msg.payload.decode("utf-8")))
        steering_a = myround(int(msg.payload.decode("utf-8")))
        on_steering(myround(int(msg.payload.decode("utf-8"))))
    if(msg.topic == Subscribe_Sp_Topic):
        print("speed " ,int(msg.payload.decode("utf-8")))
        on_drive(int(msg.payload.decode("utf-8")))
    if(msg.topic == Subscribe_Rc_Topic):
        rc_f = int(msg.payload.decode("utf-8"))
        if(rc_f == 1):
            #os.mkdir('/home/pi/Desktop/rec/fold' + str(i))
            print('starting recording')
        elif(rc_f == 0):
            i += 1
            print('stopped recording')
                 

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.subscribe(Subscribe_Rc_Topic)
client.subscribe(Subscribe_Sp_Topic)
client.subscribe(Subscribe_St_Topic)
client.connect(Server_ip, port)
client.loop_start()


camera = PiCamera()
camera.resolution = (200,66) #256,128 #200,66
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(200, 66))  #256,128
time.sleep(0.1)
#cap = cv2.VideoCapture(0)
j = 0
k = 1100
while True:
        k += 50
        print(steering_a)
        if(k > 2100):
            k = 1100
        on_steering(k)
        time.sleep(0.5)
        #p_servo_u.set_servo_pulsewidth(13, k)
        
        if (rc_f == 1):
            camera.capture(rawCapture, format = 'bgr', use_video_port = True)
            image = rawCapture.array
            #image = cv2.GaussianBlur(image, (3,3), 0)
            #image=cv2.cvtColor(image,cv2.COLOR_BGR2YUV)
            #image=cv2.Canny(image,100,200)
            #image1,dst=cv2.threshold(image1, 127,255,cv2.THRESH_BINARY)
            #image1 = cv2.resize(image, (200,66))
            cv2.imshow("Frame", image)
            #cv2.imwrite('/home/pi/Desktop/rec/fold' + str(i) + '/image' + str(j) + ':' + str(steering_a) + '.jpg',image)
            j = j + 1
            key = cv2.waitKey(1) & 0xFF
# clear the stream in preparation for the next frame
            rawCapture.truncate(0)
# if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break
            
        elif (rc_f == 0):
            j = 0
            cv2.destroyAllWindows()
         