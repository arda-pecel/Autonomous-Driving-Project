# Autonomous-Driving-Project
Autonomous driving project is an open source control software started at September 2022 and developing with Python and can be used for autonomous vehicle's.<br>
For object detection train Augelab Studio is used which is an open source image processing compiler. Dataset contains 16.185 images of 196 classes of cars.<br>
6 cameras should working together for position specification. For car detection 6 cameras are active but for line detection just 2 are enough. Raspberry Pi 4 B is used for communication with Augelab Studio and getting location informations.<br><br>

## Approach
All object detections and measurements are happened in Augelab Studio. Due to other car's position, speeds and acceleration calculations and decisions are made in Python code. Code controls single engine. (Still working for 4 engines control vehicle which does not have differential in the system)

## V0.4 Update:<br>
4 engines work together in turns. System does not have a differential. (just as a test code. not connected with powertrain yet)

## With V0.3 Update<br>
Image Compressor method added for reduce memory usage on computer. This method uses linear algebra to compress image.<br>

Other cars velocity, positions and acceleration considered in the strip changes. (still have some problems but software can seperate parking cars)<br>

System can seperate are cars in our or in opposite line.<br>

Red lights detected and vehicle moves due to traffic lights and other cars.<br>

Test code has some repetitive functions and has to be optimized.<br>

Motor control code has not integrated with engine.




https://user-images.githubusercontent.com/129625706/229360308-af5c0657-c3ca-41b0-83b0-4fa060f33281.mp4 




<br>



https://user-images.githubusercontent.com/129625706/230077784-2dff0fda-96fd-4861-bde1-69a0b92b69d0.mp4







## References
Date base provided by;<br>
 3D Object Representations for Fine-Grained Categorization
       Jonathan Krause, Michael Stark, Jia Deng, Li Fei-Fei
       4th IEEE Workshop on 3D Representation and Recognition, at ICCV 2013 (3dRR-13). Sydney, Australia. Dec. 8, 2013.
 
 Driving videos provided by;<br>
 https://www.youtube.com/watch?v=fkps18H3SXY <br>
 https://www.youtube.com/watch?v=40xZVEFVBuE
