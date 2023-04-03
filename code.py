#variables
h_distance; #horizontal distance with front car. This value comes from Augelab Studio
v_distance; #vertical distance with afront car. This value comes from Augelab Studio
h_distanceback; #horizontal distance with rear car. This value comes from Augelab Studio
v_distanceback; #vertical distance with rear car. This value comes from Augelab Studio
split_width; #line width. This value comes from Augelab Studio

#determination of velocity





#motor control

import RPi.GPIO as GPIO          
from time import sleep

in1 = 24
in2 = 23
en = 25
temp1=1

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
p=GPIO.PWM(en,1000)

p.start(25)

# r-run s-stop f-forward b-back l-low m-medium h-high    

while(1):

    x=raw_input()
    
    if x=='r':
        if(temp1==1):
         GPIO.output(in1,GPIO.HIGH)
         GPIO.output(in2,GPIO.LOW)
         x='z'
        else:
         GPIO.output(in1,GPIO.LOW)
         GPIO.output(in2,GPIO.HIGH)
         x='z'

    elif x=='s':
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        x='z'

    elif x=='f':
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        temp1=1
        x='z'

    elif x=='b':
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        temp1=0
        x='z'

    elif x=='l':
        p.ChangeDutyCycle(25)
        x='z'

    elif x=='m':
        p.ChangeDutyCycle(50)
        x='z'

    elif x=='h':
        p.ChangeDutyCycle(75)
        x='z'
        
    elif x=='e':
        GPIO.cleanup()
        print("GPIO Clean up")
        break
    
    else:
        break

        
 # split change code

        def split_change():
        if v_distance < 1000 && 0 > h_distance < 1000:
    engineleft && engineright == turn_left()
        elif 0 < v_distance < 1000 && 0 > h_distance > -1000:
    engineleft && engineright == turn_right()

    
# turn right and left code on split change 

        def turn_right():
        
        if x=='l':
        p.ChangeDutyCycle(25)
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        temp1=1
        x='m'
        time.sleep(split_width*1.5) 
        break

        elif x=='m':
        p.ChangeDutyCycle(50)
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        temp1=1
        x='h'
        time.sleep(split_width*1.5) 
        break

        elif x=='h':
        p.ChangeDutyCycle(75)
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        temp1=1
        x='h'
        time.sleep(split_width*1.5) 
        
        
        def turn_left():
        
        if x=='l':
        p.ChangeDutyCycle(25)
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        temp1=1
        x='m'
        time.sleep(split_width*1.5) 
        break

        elif x=='m':
        p.ChangeDutyCycle(50)
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        temp1=1
        x='h'
        time.sleep(split_width*1.5) 
        break

        elif x=='h':
        p.ChangeDutyCycle(75)
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        temp1=1
        x='h'
        time.sleep(split_width*1.5) 

    

# split change and emergency stop 
    
if abs(h_distance) && v_distance < 500:
    engineleft && engineright = s
elif  abs(h_distance) > 1000 && v_distance < 1000:
    engineleft && engineright = split_change()
else:
    continue
    



    
        

    
