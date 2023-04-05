# Autonomous-Driving-Project
Autonomous driving project is an open source control software started at September 2022 and developing with Python and can be used for autonomous vehicle's.<br>
For object detection train Augelab Studio is used which is an open source image processing compiler. Dataset contains 16.185 images of 196 classes of cars.<br>
6 cameras should working together for position specification. For car detection 6 cameras are active but for line detection just 2 are enough. Raspberry pi 4 is used for communication with Augelab Studio and getting location informations.<br><br>
## Latest Updates<br>
Image Compressor method added for reduce memory usage on computer. This method uses linear algebra to compress image.<br>
Other cars velocity, positions and acceleration considered in the strip changes.<br>
In current situation software catchs only one car in the screen. Also distance measurement between other cars does not working properly. (some problems at dark colour cars for night drivings) <br>
Test code has some repetitive functions and has to be optimized.


https://user-images.githubusercontent.com/129625706/229360308-af5c0657-c3ca-41b0-83b0-4fa060f33281.mp4




https://user-images.githubusercontent.com/129625706/229726333-0eb4ba89-2025-471c-807e-5e263b607dff.mp4




## Approach
All object detections and measurements are happened in Augelab Studio. Due to other car's position, speeds and acceleration calculations and decisions are made in Python code. Code controls 2 different engines located in front wheels. (Still working for 4 engines control vehicle which does not have differential in the system)


## Car Detection Results
16.185 images processed for car detections with more than 2000 iterations.<br> 
Current avg loss: less than 0.03 (less than 1.0 gives accurate results)<br> 
![chart_DENEME](https://user-images.githubusercontent.com/129625706/229354291-75f166c6-8c22-4689-8847-1fe3396c5e0e.png)<br>

Some car predictions;
![1](https://user-images.githubusercontent.com/129625706/229356596-dfc03afd-a811-4e46-acdd-c22ac58a59e6.png)
![2](https://user-images.githubusercontent.com/129625706/229356600-8704bdbe-db60-4cdc-b617-0d4626f86a2b.png)
![3](https://user-images.githubusercontent.com/129625706/229356603-6e2bdd0c-3a76-4c4a-8b60-0a8ef5de560d.png)
![4](https://user-images.githubusercontent.com/129625706/229356604-e451bcf2-273b-49f5-a7c2-d490be44f565.png)
![5](https://user-images.githubusercontent.com/129625706/229356607-ea5f8870-f6d1-432b-8029-ce2666f1ae5f.png)


## Line Detection Results
Vehicle trying to center the strip.<br> 
![Ekran görüntüsü 2023-04-02 161745](https://user-images.githubusercontent.com/129625706/229355433-aab0bbc2-2eec-4f39-bc31-86cdb6fb1261.png)<br>

Also traffic lights detections achieved succesfully.
![resim_2023-04-02_185938212](https://user-images.githubusercontent.com/129625706/229364476-4002fe8a-a403-40c1-88f3-d4b21fa2bc31.png)



## References
Date base provided by;<br>
 3D Object Representations for Fine-Grained Categorization
       Jonathan Krause, Michael Stark, Jia Deng, Li Fei-Fei
       4th IEEE Workshop on 3D Representation and Recognition, at ICCV 2013 (3dRR-13). Sydney, Australia. Dec. 8, 2013.
 
 Driving videos provided by;<br>
 https://www.youtube.com/watch?v=fkps18H3SXY <br>
 https://www.youtube.com/watch?v=40xZVEFVBuE
