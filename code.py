#variables
h_distance; #horizontal distance with front car. This value comes from Augelab Studio
v_distance; #vertical distance with afront car. This value comes from Augelab Studio
h_distanceback; #horizontal distance with rear car. This value comes from Augelab Studio
v_distanceback; #vertical distance with rear car. This value comes from Augelab Studio
split_width; #line width. This value comes from Augelab Studio

#determination of velocity ***THIS PART OF THE CODE DOES NOT WORK PROPERLY. IN NEXT UPDATE SECOND CODE WILL BE TRIED***

values = [[h_distance],[v_distance],[h_distanceback],[v_distanceback]]
last_values = [0,0]
last_velocity = 0
delta = 0.1  # Will need to play with this value.
terminal_velocity = None
for pos, time in values:
    velocity = (pos - last_values[0]) / (time - last_values[1])
    if abs(velocity - last_velocity) < delta:
        terminal_velocity = velocity
        break
    last_values = [pos, time]
    last_velocity = velocity

#second code will be tried

import numpy as np

# define function to calculate distance between two points
def distance(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

# set initial values
last_pos = None
last_time = None
other_car_positions = [(x1, y1), (x2, y2), ...] # list of positions of other cars. This values will be getted from Augelab Studio.
other_car_speeds = [v1, v2, ...] # list of speeds of other cars. This values will be getted from Augelab Studio.

# get current position and time
current_pos = (my_pos_x, my_pos_y) # your car's current position
current_time = time.time() # current time in seconds

# calculate distances between your car and the other cars
distances = []
for pos in other_car_positions:
    distances.append(distance(current_pos, pos))

# calculate relative velocities between your car and the other cars
if last_pos is not None and last_time is not None:
    dt = current_time - last_time
    rel_velocities = []
    for i in range(len(other_car_speeds)):
        rel_velocities.append((distances[i] - distance(last_pos[i], other_car_positions[i])) / dt)
else:
    rel_velocities = [0] * len(other_car_speeds)

# calculate your velocity
your_speed = my_speed # your car's speed
your_velocity = your_speed - sum(rel_velocities)

# update last_pos and last_time
last_pos = other_car_positions
last_time = current_time
    
    
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
    

# improve, decrease speed on normal driving

if velocity>100:
    if abs(h_distance) && v_distance > 1000:
    p.ChangeDutyCycle(50)
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        temp1=1
        x='h'
    elif abs(h_distance) && v_distance > 500:
        continue
    else:
        p.ChangeDutyCycle(50)
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        temp1=1
        x='m'
        
elif velocity>50:
    if abs(h_distance) && v_distance > 500:
    p.ChangeDutyCycle(50)
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        temp1=1
        x='m'
    elif abs(h_distance) && v_distance > 250:
        continue
    else:
        p.ChangeDutyCycle(50)
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        temp1=1
        x='l'
else:
    if abs(h_distance) && v_distance > 125:
    p.ChangeDutyCycle(50)
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        temp1=1
        continue
    elif abs(h_distance) && v_distance > 50:
        p.ChangeDutyCycle(50)
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        temp1=1
        continue
    else:
        p.ChangeDutyCycle(50)
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        temp1=1
        x='s'
        break

    
    




    
        

    
