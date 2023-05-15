#variables. This values come from Augelab Studio

import pandas as pd

# Read the CSV file and store it in a variable
h_distance = pd.read_csv('data1.csv') #horizontal distance with front car. 
v_distance = pd.read_csv('data2.csv') #vertical distance with afront car.
h_distanceback = pd.read_csv('data12.csv') #horizontal distance with rear car.
v_distanceback = pd.read_csv('data23.csv') #vertical distance with rear car.


import json

# Read the JSON file and store it in a variable
with open('split_width.json', 'r') as f:
    split_width = json.load(f)

    

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
other_car_positions = [(x1, y1), (x2, y2), (x3, y3), (x4, y4), (x5, y5), (x6, y6)] # list of positions of other cars. This values will be getted from Augelab Studio.
other_car_speeds = [v1, v2, v3, v4, v5, v6] # list of speeds of other cars. This values will be getted from Augelab Studio.

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
    
    
# Simulate getting positions and speeds of other cars from Augelab Studio
def get_other_cars():
    return [{'position': h_distance, v_distance, 'speed': speed}

# Simulate sending control signals to adjust car's speed and steering
def send_control_signals(speed, steering):
    print(f'Speed: {speed}, Steering: {steering}')    
     

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
    

# increase, decrease speed on normal driving

def my_speed():           
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
        

#This code should be also tried for speed control.

while True:
    other_cars = get_other_cars()
    # Assuming your own car is at position 0 with an initial speed of 0
    my_position = 0
    my_speed = 0
    for car in other_cars:
        relative_position = car['position'] - my_position
        relative_speed = car['speed'] - my_speed
        # Decide whether to increase or decrease speed based on relative position and speed
        if relative_position > 50:
            my_speed += 5
        elif relative_position < 30:
            my_speed -= 5
        # Decide whether to change lanes based on relative position and speed
        if relative_position < 10 and relative_speed < 0:
            # Change lanes to the left
            steering = -1
        elif relative_position > 10 and relative_speed > 0:
            # Change lanes to the right
            steering = 1
        else:
            steering = 0
    send_control_signals(my_speed, steering)
    time.sleep(0.1)  # Pause for a short time before checking other cars again
            
            
import random

class Car:
    def __init__(self, pos, speed):
        self.pos = pos
        self.speed = speed
        self.accel = 0
    
    def update_pos(self, dt):
        self.pos += self.speed * dt + 0.5 * self.accel * dt ** 2
        self.speed += self.accel * dt
    
    def update_accel(self, other_cars):
        # determine the desired acceleration based on the other cars' positions and speeds
        desired_accel = 0
        if len(other_cars) > 0:
            closest_car = min(other_cars, key=lambda car: abs(car.pos - self.pos))
            distance_to_closest_car = closest_car.pos - self.pos
            relative_speed_to_closest_car = closest_car.speed - self.speed
            if distance_to_closest_car < 30:
                # brake if too close to the closest car
                desired_accel = -0.5 * (relative_speed_to_closest_car ** 2) / distance_to_closest_car
            else:
                # accelerate to reach desired speed
                desired_speed = min(closest_car.speed - 10, 30)
                desired_accel = (desired_speed - self.speed) / dt
        
        # add some noise to the acceleration to simulate human driving behavior
        self.accel = desired_accel + random.uniform(-1, 1)
        self.accel = max(-2, min(self.accel, 2))
    
    def update_speed(self, desired_speed):
        # adjust the speed based on the desired acceleration and a maximum acceleration limit
        max_accel = 1
        self.accel = max(-max_accel, min(self.accel, max_accel))
        self.speed += self.accel * dt
        self.speed = max(0, min(self.speed, desired_speed))
    
    def change_lane(self, other_cars):
        # change lane randomly with a probability of 0.1%
        if random.random() < 0.001:
            self.pos += random.choice([-10, 10])
    
# initialize the cars
num_cars = 10
cars = [Car(i * 50, random.uniform(20, 30)) for i in range(num_cars)]

# simulate the cars for 1000 seconds
dt = 0.1
for t in range(10000):
    for i, car in enumerate(cars):
        other_cars = [c for j, c in enumerate(cars) if j != i]
        car.update_accel(other_cars)
        car.update_speed(30)
        car.update_pos(dt)
        car.change_lane(other_cars)
    
 
#Considering acceleration           
            
import numpy as np

class Car:
    def __init__(self, pos, speed, acc):
        self.pos = pos   # current position of the car
        self.speed = speed   # current speed of the car
        self.acc = acc   # current acceleration of the car

    def update(self, dt):
        self.speed += self.acc * dt   # update the speed based on the acceleration
        self.pos += self.speed * dt   # update the position based on the speed

class Controller:
    def __init__(self, car, others, dt, target_speed):
        self.car = car   # the controlled car
        self.others = others   # the other cars
        self.dt = dt   # time step
        self.target_speed = target_speed   # target speed of the controlled car

    def update(self):
        # Compute the acceleration of the controlled car based on the positions and speeds of the other cars
        acc = self.compute_acceleration()

        # Limit the acceleration to avoid abrupt changes in speed
        max_acc = 5.0  # maximum acceleration
        min_acc = -5.0  # maximum deceleration
        acc = max(min(acc, max_acc), min_acc)

        # Update the acceleration of the controlled car
        self.car.acc = acc

        # Update the position and speed of the controlled car
        self.car.update(self.dt)

    def compute_acceleration(self):
        # Compute the position and speed differences between the controlled car and the other cars
        pos_diff = [other.pos - self.car.pos for other in self.others]
        speed_diff = [other.speed - self.car.speed for other in self.others]

        # Compute the distance and relative speed between the controlled car and the other cars
        distance = np.sqrt(np.sum(np.square(pos_diff), axis=1))
        rel_speed = np.array(speed_diff) - self.car.speed

        # Compute the time to collision between the controlled car and the other cars
        ttc = distance / (rel_speed + 1e-5)

        # Compute the desired speed based on the target speed and the distance to the nearest car
        min_distance = np.min(distance)
        desired_speed = self.target_speed - min_distance / 10.0

        # Compute the desired acceleration based on the desired speed and the current speed of the controlled car
        acc = (desired_speed - self.car.speed) / self.dt

        # Compute the lateral deviation of the controlled car from the center of the lane
        lateral_deviation = self.car.pos[1]

        # If there is a car in front of the controlled car and the lateral deviation is too large,
        # adjust the acceleration to move the car towards the center of the lane
        if min_distance < 50.0 and abs(lateral_deviation) > 1.0:
            acc += np.sign(lateral_deviation) * 1.0

        return acc
   
            
# determine cars in opposite line     
            
import cv2

# Define the minimum and maximum areas for car detection
min_area = 100
max_area = 5000

#input image
img = cv2.imread("camera_feed.jpg")


# Detect contours in the front image
front_gray = cv2.cvtColor(front_img, cv2.COLOR_BGR2GRAY)
_, front_thresh = cv2.threshold(front_gray, 100, 255, cv2.THRESH_BINARY)
front_contours, _ = cv2.findContours(front_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Detect contours in the back image
back_gray = cv2.cvtColor(back_img, cv2.COLOR_BGR2GRAY)
_, back_thresh = cv2.threshold(back_gray, 100, 255, cv2.THRESH_BINARY)
back_contours, _ = cv2.findContours(back_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Combine the front and back contours into a single list
all_contours = front_contours + back_contours

# Loop over all the detected contours and check if they meet the size criteria
for contour in all_contours:
    area = cv2.contourArea(contour)
    if area < min_area or area > max_area:
        continue
    
    # Get the bounding box of the contour
    x, y, w, h = cv2.boundingRect(contour)
    
    # Calculate the centroid of the contour
    cx = x + w/2
    cy = y + h/2
    
    car_center_x = img.shape[1] / 2  # Assuming the car is driving in the center of the image
    if cx < car_center_x - w/2 or cx > car_center_x + w/2:
        print("Car in opposite lane detected at ({}, {})".format(cx, cy))
    else:
        print("Car in your lane detected at ({}, {})".format(cx, cy))

          

# Initialize the video camera
cap = cv2.VideoCapture(0)

while True:
    # Read in a frame from the video stream
    ret, frame = cap.read()

    # Perform object detection on the frame
    
    # Checking red traffic lights
    if red_light_detected:
        # Slowly stop the vehicle
        for i in range(10):
            slow_down()
            time.sleep(0.1)
            
    # Check if a green traffic light is detected and there are no vehicles in front of you
    if green_light_detected and no_vehicle_in_front:
        # Increase the speed of the vehicle step by step
        for i in range(10):
            speed_up()
            time.sleep(0.1)
    
    # Check if there are any vehicles in front of you while waiting at a red light
    if red_light_detected and vehicle_in_front:
        # Stop the vehicle if a vehicle is detected in front
        stop()
            
    # Display the video stream with the object detection overlay
    cv2.imshow("Video Stream", frame)
    
    # Exit the program if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video camera and close all windows
cap.release()
cv2.destroyAllWindows()


import paho.mqtt.client as mqtt

# Define the MQTT broker address and port
broker_address = "192.168.1.100"
broker_port = 1883

# Define the topics for each camera
front_left_topic = "car/camera/front/left"
front_center_topic = "car/camera/front/center"
front_right_topic = "car/camera/front/right"
back_left_topic = "car/camera/back/left"
back_center_topic = "car/camera/back/center"
back_right_topic = "car/camera/back/right"

# Create an MQTT client instance
client = mqtt.Client()

# Connect to the MQTT broker
client.connect(broker_address, broker_port)

# Define a function to handle incoming messages
def on_message(client, userdata, message):
    # Process the message
    print("Received message on topic:", message.topic)
    print("Message payload:", message.payload)

# Subscribe to the topics for each camera
client.subscribe(front_left_topic)
client.subscribe(front_center_topic)
client.subscribe(front_right_topic)
client.subscribe(back_left_topic)
client.subscribe(back_center_topic)
client.subscribe(back_right_topic)

# Set the on_message callback function
client.on_message = on_message

# Start the MQTT loop to listen for incoming messages
client.loop_forever()
            
# acceleration due to traffic lights and other cars

# Create a class for the autonomous vehicle
class AutonomousVehicle:
    def __init__(self):
        self.position = 0
        self.speed = 0
        self.acceleration = 0
        self.braking_distance = 0
        self.traffic_light_color = "green"
        self.distance_to_traffic_light = 100
        self.other_cars = []

    def update_position(self, time_interval):
        # Update the vehicle's position based on its speed and acceleration
        self.position += (self.speed * time_interval) + (0.5 * self.acceleration * time_interval ** 2)

    def update_speed(self, time_interval):
        # Update the vehicle's speed based on its acceleration
        self.speed += self.acceleration * time_interval

    def update_acceleration(self):
        # Update the vehicle's acceleration based on its distance to the traffic light and other cars
        if self.traffic_light_color == "red":
            # If the traffic light is red, the vehicle should slow down and eventually stop at the traffic light
            if self.distance_to_traffic_light > self.braking_distance:
                self.acceleration = -1.0
            else:
                self.acceleration = 0.0
        else:
            # If the traffic light is green, the vehicle can accelerate if there are no other cars in the way
            if len(self.other_cars) == 0:
                self.acceleration = 1.0
            else:
                # If there are other cars in the way, the vehicle should slow down to avoid collision
                closest_car_distance = min([car.position - self.position for car in self.other_cars])
                if closest_car_distance > self.braking_distance:
                    self.acceleration = 1.0
                else:
                    self.acceleration = -1.0

    def update_traffic_light(self):
        # Randomly change the traffic light color every 10 seconds
        if time.time() % 10 == 0:
            self.traffic_light_color = random.choice(["red", "green"])

    def update_other_cars(self):
        # Randomly add or remove other cars from the road every 5 seconds
        if time.time() % 5 == 0:
            if random.random() < 0.5:
                self.other_cars.append(AutonomousVehicle())
            else:
                self.other_cars.pop()

    def run(self):
        # Run the simulation for 60 seconds
        for i in range(60):
            # Update the traffic light color and other cars
            self.update_traffic_light()
            self.update_other_cars()

            # Update the vehicle's acceleration, speed, and position
            self.update_acceleration()
            self.update_speed(1)
            self.update_position(1)

            # Print the current status of the vehicle
            print("Time: {}s, Position: {}m, Speed: {}m/s, Acceleration: {}m/s^2, Traffic Light: {}, Other Cars: {}".format(i, self.position, self.speed, self.acceleration, self.traffic_light_color, len(self.other_cars)))

# Create an instance of the AutonomousVehicle class and run the simulation
vehicle = AutonomousVehicle()
vehicle.run()
  
            
#4 engines update
            
import pigpio

# Initialize the pigpio library to control the GPIO pins
pi = pigpio.pi()

# Define the pins connected to each of the four motors
motor_pins = [
    [12, 13],  # Front left motor pins
    [16, 17],  # Front right motor pins
    [20, 21],  # Rear left motor pins
    [24, 25]   # Rear right motor pins
]

# Set the PWM frequency and range for the motors
pwm_frequency = 1000
pwm_range = 1000

# Set the maximum speed of the motors
max_speed = 0.2

# Set the speed of each motor to zero initially
motor_speeds = [0, 0, 0, 0]

def set_motor_speeds(motor_speeds):
    """
    Sets the speed of each motor based on the input list of speeds.
    """
    for i in range(4):
        pi.set_PWM_dutycycle(motor_pins[i][0], int(motor_speeds[i] * pwm_range))
        pi.set_PWM_dutycycle(motor_pins[i][1], 0)

def update_motor_speeds(steering_angle):
    """
    Updates the speed of each motor based on the input steering angle.
    """
    # Compute the speed of the outer and inner wheels based on the steering angle
    outer_speed = max_speed
    inner_speed = max_speed * (1 - 0.5 * abs(steering_angle))

    # Compute the speed of each motor based on the outer and inner wheel speeds
    fl_speed = inner_speed
    fr_speed = outer_speed
    rl_speed = inner_speed
    rr_speed = outer_speed

    # Set the speed of each motor
    motor_speeds[0] = fl_speed
    motor_speeds[1] = fr_speed
    motor_speeds[2] = rl_speed
    motor_speeds[3] = rr_speed

# Test the motor control by ramping up the speed of each motor one by one
for i in range(4):
    motor_speeds[i] = 0.2
    set_motor_speeds(motor_speeds)
    time.sleep(2)

# Update the motor speeds based on a steering angle of 30 degrees to the right
update_motor_speeds(np.deg2rad(30))
set_motor_speeds(motor_speeds)
time.sleep(2)

# Set the motor speeds to zero and stop the pigpio library
motor_speeds = [0, 0, 0, 0]
set_motor_speeds(motor_speeds)
pi.stop()
            

#back wheels turn on half angle of front wheels in turns            
            
def update_motor_speeds(steering_angle):
    """
    Updates the speed of each motor based on the input steering angle.
    """
    # Compute the speed of the outer and inner wheels based on the steering angle
    outer_speed = max_speed
    inner_speed = max_speed * (1 - 0.9 * abs(steering_angle))

    # Compute the speed of each motor based on the outer and inner wheel speeds
    fl_speed = inner_speed
    fr_speed = outer_speed
    rl_speed = inner_speed * 0.1
    rr_speed = outer_speed * 0.1

    # Set the speed of each motor
    motor_speeds[0] = fl_speed
    motor_speeds[1] = fr_speed
    motor_speeds[2] = rl_speed
    motor_speeds[3] = rr_speed

            
#wheels turnings
  
    class AutonomousDriving:
    def __init__(self, front_left_engine, front_right_engine, back_left_engine, back_right_engine):
        self.front_left_engine = front_left_engine
        self.front_right_engine = front_right_engine
        self.back_left_engine = back_left_engine
        self.back_right_engine = back_right_engine
        
        self.speed = 0
        self.reverse = False
        
    def set_speed(self, speed):
        self.speed = speed
        
        if self.speed > 120:
            self.back_left_engine.set_direction(self.front_left_engine.get_direction() * 0.1)
            self.back_right_engine.set_direction(self.front_right_engine.get_direction() * 0.1)
        else:
            self.back_left_engine.set_direction(self.front_left_engine.get_direction())
            self.back_right_engine.set_direction(self.front_right_engine.get_direction())
            
    def set_reverse(self, reverse):
        self.reverse = reverse
        
        if self.reverse:
            self.back_left_engine.set_direction(-self.front_left_engine.get_direction() * 0.5)
            self.back_right_engine.set_direction(-self.front_right_engine.get_direction() * 0.5)
        else:
            self.back_left_engine.set_direction(self.front_left_engine.get_direction())
            self.back_right_engine.set_direction(self.front_right_engine.get_direction())

