Romi-O


[![Watch the video](https://img.youtube.com/vi/ucUNd86v_RI/0.jpg)](https://youtu.be/ucUNd86v_RI?si=cy1BHj5UCIDBjYc2)

#CAD ReadMe
The Romi robot consists of a plastic base, bought from Pololu, upon which two DC motors are mounted. A laser-cut acrylic piece is attached to the base via multiple metal stand offs (provided by our professor). The Nucleo L476 sits atop this acrylic piece.  

![image](https://github.com/user-attachments/assets/ed11c48e-119e-4928-b386-a1c8c50f31ee)
A 3d printed fixture was designed to attach the IR sensor array and two limit switches to the front of the Romi. A CAD model approximation of the IR array and the CAD model of the mount are provided in SolidWorks format.

Electrical ReadMe

The overall electrical layout:
![image](https://github.com/user-attachments/assets/aab5baaf-0229-489f-bc56-d4008f9c643d)

BNO055 IMU Sensor  
![image](https://github.com/user-attachments/assets/2d70a372-22b0-426c-9ca0-f4dde21e9a8c)

Purpose: Measures yaw, pitch, and roll for navigation. Used for rotational feedback and precise movement adjustments. 
 
Infrared (IR) Sensors (ITR2001)   

![image](https://github.com/user-attachments/assets/ab3d44ad-9c32-4259-9182-7c4955271e84)

Purpose: Line-following detection. Implemented within an array of five sensors for calculating the total line deviation by using weights. 
Thresholds: Includes ceiling and floor thresholds for signal normalization. Each sensor was experimentally tested until each of the thresholds were found.  
 
Bump Sensors (KW12)  

![image](https://github.com/user-attachments/assets/9984de5b-84b2-4767-90d0-3cdbed049417)

Purpose: Detect collisions with objects.

The Romi's interaction with the sensor data is facilitated by the STM32L476 Microcontroller. The array of infared sensors allow the robot to calculate a total deviation away from the line. This was done by attaching weights to each infared sensor, and the analog values were read in from the ADC. A BNO055 inertial measurement unit (IMU) is utilized for orientation tracking, providing yaw, pitch, and roll data critical for maintaining stability and accurate directional acceleration control. Additionally, two KW12 bump sensors are mounted on the front corners of the robot to detect collisions with obstacles, ensuring safety and triggering corrective maneuvers. The sensors receive power from the Nucleo board itself while the motors receive power from the Romi's Power Distrubution Board. The robot's mobility is powered by two DC motors, each controlled by an L6206 motor driver for speed and direction control. Each motor is paired with an encoder to provide feedback on wheel speed, essential for closed-loop control and precise velocity adjustments. The motors drive the wheels via PWM signals, with encoders measuring rotational speed to maintain consistent motion. Further documentation of each of the sensors can be found in Documentation/Electrical.
