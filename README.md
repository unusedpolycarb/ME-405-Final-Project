# Romi-O


[![Watch the video](https://img.youtube.com/vi/ucUNd86v_RI/0.jpg)](https://youtu.be/ucUNd86v_RI?si=cy1BHj5UCIDBjYc2)

## Mechanical Design
The Romi robot consists of a plastic base, bought from Pololu, upon which two DC motors are mounted. A laser-cut acrylic piece is attached to the base via multiple metal stand offs (provided by our professor). The Nucleo L476 sits atop this acrylic piece.  

![image](https://github.com/user-attachments/assets/ed11c48e-119e-4928-b386-a1c8c50f31ee)
A 3d printed fixture was designed to attach the IR sensor array and two limit switches to the front of the Romi. A CAD model approximation of the IR array and the CAD model of the mount are provided in SolidWorks format.

## Electrical Design

### The overall electrical layout:
![image](https://github.com/user-attachments/assets/aab5baaf-0229-489f-bc56-d4008f9c643d)

### BNO055 IMU Sensor  
![image](https://github.com/user-attachments/assets/2d70a372-22b0-426c-9ca0-f4dde21e9a8c)

Purpose: Measures yaw, pitch, and roll for navigation. Used for rotational feedback and precise movement adjustments. 
 
### Infrared (IR) Sensors (ITR2001)   

![image](https://github.com/user-attachments/assets/ab3d44ad-9c32-4259-9182-7c4955271e84)

Purpose: Line-following detection. Implemented within an array of five sensors for calculating the total line deviation by using weights. 
Thresholds: Includes ceiling and floor thresholds for signal normalization. Each sensor was experimentally tested until each of the thresholds were found.  
 
### Bump Sensors (KW12)  

![image](https://github.com/user-attachments/assets/9984de5b-84b2-4767-90d0-3cdbed049417)

Purpose: Detect collisions with objects.

The Romi's interaction with the sensor data is facilitated by the STM32L476 Microcontroller. The array of infared sensors allow the robot to calculate a total deviation away from the line. This was done by attaching weights to each infared sensor, and the analog values were read in from the ADC. A BNO055 inertial measurement unit (IMU) is utilized for orientation tracking, providing yaw, pitch, and roll data critical for maintaining stability and accurate directional acceleration control. Additionally, two KW12 bump sensors are mounted on the front corners of the robot to detect collisions with obstacles, ensuring safety and triggering corrective maneuvers. The sensors receive power from the Nucleo board itself while the motors receive power from the Romi's Power Distrubution Board. The robot's mobility is powered by two DC motors, each controlled by an L6206 motor driver for speed and direction control. Each motor is paired with an encoder to provide feedback on wheel speed, essential for closed-loop control and precise velocity adjustments. The motors drive the wheels via PWM signals, with encoders measuring rotational speed to maintain consistent motion. Further documentation of each of the sensors can be found in Documentation/Electrical.

## Software Design
The program is structured around a cooperative task-based architecture. There are three tasks: the controller task, motor task, and user input task. Tasks share information with each other through a share (a positive integer) and a queue of positive integers. Each task has multiple states that it cycles through.

The limitations on what information each task can share were very difficult to work with. The solution our team came up with was to create flags, which would be placed in the “share” variable, and put additional relevant information for that flag in the queue. Below is a list of possible flags and the required information:
 
![image](https://github.com/user-attachments/assets/d1f1923e-6ccf-4858-bcaf-01e1ae9419c2)

### Controller Task:
The controller task is specifically tuned to each challenge. In this case, it is specifically tuned to every step in the maze. As such, there are 22 states which progress in sequence. Each state will only exit upon a certain condition being completed. For example, the program will only exit the line following state (2) if the it senses the bump sensors were pressed. The full sequence is shown below:

![image](https://github.com/user-attachments/assets/18a6a77f-9669-4266-8c9e-baf96d4a77f0)

### Motor Task:
The motor task keeps track of two state variables, one for the right motor and one for the left motor. For each motor, three states are possible. The first corresponds to static controls which only have to be called once. Commands such as set_motor_duty and set_motor_speed fall into this category because only one call to the motor driver is needed. State 2 refers to our trapezoidal velocity profile state. In this state multiple calls to the motor driver are made, as the motor velocity ramps toward the target velocity. State 3 implements our PI control for the Romi yaw rate, setting corrected motor speeds each time it polls the IMU.

![image](https://github.com/user-attachments/assets/a3f535a0-c0be-44f0-b186-fffae2fda843)

### User Input Task:
The user input task is very simple with only two states for every possible button (of which there are three). Upon noticing that the button has been pressed, it will move to state 2 where it attempts to raise the appropriate flag repeatedly. If it successfully raises the corresponding flag, it will return to state 1 and continue polling for button presses.

![image](https://github.com/user-attachments/assets/caeb97ff-0f13-496d-a720-e294e8075bfa)



