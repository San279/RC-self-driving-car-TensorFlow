import argparse
import sys
import time
import os
import cv2
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import tflite_runtime.interpreter as tflite
import RPi.GPIO as GPIO
import numpy as np
import utils
from picamera.array import PiRGBArray
from picamera import PiCamera
import pigpio

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
p_servo = pigpio.pi()
p_servo_u = pigpio.pi()
p_servo.set_mode(12, pigpio.OUTPUT)
p_servo_u.set_mode(18, pigpio.OUTPUT)
p_servo.set_PWM_frequency(12, 50)
p_servo_u.set_PWM_frequency(18, 50)

GPIO.setup(13,GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
p1 = GPIO.PWM(19, 50)
p2 = GPIO.PWM(13 ,50)
GPIO.setup(20,GPIO.OUT)
GPIO.setup(21,GPIO.OUT)
GPIO.setup(2,GPIO.OUT)
GPIO.setup(3,GPIO.OUT)
p1.start(0)
p2.start(0)

GPIO.setup(14,GPIO.OUT)
GPIO.setup(15,GPIO.IN)

def On_M1(pwm): #servo motor
       p_servo.set_servo_pulsewidth(12, pwm)
       ultra_pwm = calibrate_sensor(pwm)
       if(ultra_pwm <= 1150):
            ultra_pwm = 1150
       elif(ultra_pwm >= 2000):
            ultra_pwm = 2000
       p_servo_u.set_servo_pulsewidth(18, ultra_pwm)

def On_M2(pwm): #Tur8 on speed motor for both
       if(pwm >= 10):
           GPIO.output(20,GPIO.HIGH)#IN1
           GPIO.output(21,GPIO.LOW)#IN2
           GPIO.output(2,GPIO.HIGH)#IN1
           GPIO.output(3,GPIO.LOW)#IN2
       if(9 < pwm < -9):
           GPIO.output(20,GPIO.HIGH)#IN1
           GPIO.output(21,GPIO.HIGH)#IN2
           GPIO.output(2,GPIO.HIGH)#IN1
           GPIO.output(3,GPIO.HIGH)#IN2
       elif(pwm <= -10):
           GPIO.output(20,GPIO.LOW)#IN1
           GPIO.output(21,GPIO.HIGH)#IN2
           GPIO.output(2,GPIO.LOW)#IN1
           GPIO.output(3,GPIO.HIGH)#IN2
       p2.ChangeDutyCycle(pwm) #motor one percentage speed
       p1.ChangeDutyCycle(pwm) #motor two percentage speed

def myround(x, base=50):    #rounding for servo motor
    return base * round(x/base)

def scaledata(x):
    return float(x * 1000) + 1100

def calibrate_sensor(x):
    y = (x - 1100)/1000
    y = float("{0:.2f}".format(y))
    y = abs(y-1)
    y = (y * 1000) + 1100
    y = myround(y)
    return y

def dist_sensor():

    GPIO.output(14, True) #trigger transmitter
    time.sleep(0.0001) #send 8 pulses
    GPIO.output(14, False) #stop transmitting
    send= time.time()
    receive = time.time()
    while GPIO.input(15) == 0:
            send = time.time()
            if(abs(send - receive)> 0.05):
                break
    # save time of arrival
    while GPIO.input(15) == 1:
            receive = time.time()
            #print("stuck at second loop")
    time.sleep(0.005)
    tot = receive - send
    dist = tot/0.000058
    dist =abs(np.round_(dist,2))
    print(dist)
    return dist


#change the path of EffcientDet_Lite0 model
def run(model:'/home/pi/Desktop/testlite/Edet744s.tflite', camera_id: 0, width: 66, height: 200, num_threads: 4,
        ):

  #i = 70
  #os.mkdir('/home/pi/Desktop/rec/fold' + str(i))
  interpreter = tflite.Interpreter(model_path="/home/pi/Desktop/testlite/model(f00184r).tflite") ##change the path of PilotNet model
  interpreter.allocate_tensors()

  input_details = interpreter.get_input_details()
  output_details = interpreter.get_output_details()
  print(input_details)
  print(output_details)
  height = input_details[0]['shape'][1]
  width = input_details[0]['shape'][2]
  # Variables to calculate FPS
  counter, fps = 0, 0
  start_time = time.time()

  # Start capturing video input from the camera
  camera = PiCamera()
  camera.resolution = (208,80) #256,128
  camera.framerate = 32
  rawCapture = PiRGBArray(camera, size=(208, 80))  #256,128
  time.sleep(1)

  # Visualization parameters
  row_size = 20  # pixels
  left_margin = 24  # pixels
  text_color = (0, 0, 255)  # red
  font_size = 1
  font_thickness = 1
  fps_avg_frame_count = 10

  # Initialize the object detection model
  base_options = core.BaseOptions(
      file_name=model, num_threads=num_threads)
  detection_options = processor.DetectionOptions(
      max_results=3, score_threshold=0.55)
  options = vision.ObjectDetectorOptions(
      base_options=base_options, detection_options=detection_options)
  detector = vision.ObjectDetector.create_from_options(options)
  #chars = Set('0123456789$,')
  j = 0

  # Continuously capture images from the camera and run inference
  for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    input_image = frame.array
    j += 1

    distance =abs(dist_sensor())
    counter += 1
    #image = cv2.flip(rgb_image, 1)

    # Convert the image from BGR to RGB as required by the TFLite model.
    rgb_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB)

    yuv_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2YUV)

    yuv_image = yuv_image[35:80, :, :]
    yuv_image = cv2.resize(yuv_image,(200,66))
    
    PilotNet_show = cv2.GaussianBlur(yuv_image,(3,3),0)
    
    imgeq=PilotNet_show/255.0

    imgre = imgeq.reshape(1,66,200,3)
    imgre = (np.float32(imgre))

    interpreter.set_tensor(input_details[0]['index'], imgre)

    interpreter.invoke()

    classes = interpreter.get_tensor(output_details[0]['index'])
    roundval = np.round_(abs(classes),2)
    print(classes)
    if(roundval > 1.0):
       roundval = 1.0
    roundval = scaledata(roundval)
    On_M1(roundval) #Steering servo

    # Create a TensorImage object from the RGB image.
    input_tensor = vision.TensorImage.create_from_array(rgb_image)

    # Run object detection estimation using the model.
    detection_result = detector.detect(input_tensor)

    # Draw keypoints and edges on input image
    rgb_image, lights, *c = utils.visualize(rgb_image, detection_result)
    print(lights)
    if(lights == 'yellow' or 30 <= distance <= 40):
        print('slowing')
        On_M2(40)
    if(lights =='red' or distance < 30):
        On_M2(0)
        print('stopped')
        time.sleep(2.5)
        #On_M1(1600)
    else:
        On_M2(49)
    #cv2.imwrite('/home/pi/Desktop/rec/fold' + str(i) +
    #            '/image' + str(j) + ':' + str(roundval) + '_'
    #            + str(distance) + 'cm' + '.jpg',imgeqs)
    #print(lights)
    # Calculate the FPS
    if counter % fps_avg_frame_count == 0:
      end_time = time.time()
      fps = fps_avg_frame_count / (end_time - start_time)
      start_time = time.time()

    # Show the FPS
    fps_text = 'FPS = {:.1f}'.format(fps)
    text_location = (left_margin, row_size)
    #put text for efficientDet
    cv2.putText(rgb_image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)
    #put text for 
    cv2.putText(PilotNet_show, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                font_size, text_color, font_thickness)

    cv2.imshow('object_detector', rgb_image)
    cv2.imshow('Pilotnet', PilotNet_show)
            
    key = cv2.waitKey(1) & 0xFF

    # Stop the program if the ESC key is pressed.
    rawCapture.truncate(0)
    if key == ord("q"):
        break

  cap.release()
  cv2.destroyAllWindows()
  On_M2(0) #turn off the motor
  On_M1(1600) #return steering to straight

def main():
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model',
      help='/home/pi/Desktop/testlite/Edet744s.tflite', #EfficientDet D0 model object detector
      required=False,
      default='/home/pi/Desktop/testlite/Edet744s.tflite')
  parser.add_argument(
      '--cameraId', help='0', required=False, type=int, default=0)
  parser.add_argument(
      '--frameWidth',
      help='200',
      required=False,
      type=int,
      default=640)
  parser.add_argument(
      '--frameHeight',
      help='66',
      required=False,
      type=int,
      default=480)
  parser.add_argument(
      '--numThreads',
      help='Number of CPU threads to run the model.',
      required=False,
      type=int,
      default=4)
  args = parser.parse_args()

  run(args.model, int(args.cameraId), args.frameWidth, args.frameHeight,
      int(args.numThreads))


if __name__ == '__main__':
  main()


