# Autonomous-Driving-Project
Autonomous driving project is a control software started at September 2022 and developing with Python and can be used for autonomous vehicle's.<br>
For object detection train Augelab Studio is used which is an open source image processing compiler. Dataset contains 16.185 images of 196 classes of cars.<br>
6 cameras should working together for position specification. For car detection 6 cameras are active but for line detection just 2 are enough. Raspberry pi 4 is used for communication with Augelab Studio and getting location informations.

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
![Ekran görüntüsü 2023-04-02 161745](https://user-images.githubusercontent.com/129625706/229355433-aab0bbc2-2eec-4f39-bc31-86cdb6fb1261.png)

## References
Date base provided by;<br>
 3D Object Representations for Fine-Grained Categorization
       Jonathan Krause, Michael Stark, Jia Deng, Li Fei-Fei
       4th IEEE Workshop on 3D Representation and Recognition, at ICCV 2013 (3dRR-13). Sydney, Australia. Dec. 8, 2013.
 
 Driving video provided by;<br>
 https://www.youtube.com/watch?v=fkps18H3SXY
