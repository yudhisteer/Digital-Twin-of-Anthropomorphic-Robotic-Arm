# 6dof Robotic Arm Simulation

## Abstract

## Plan of Action

### 1. Industrial Robots
Most industrial robots are mechanical arms controlled by electric servo motors and programmed to perform a specific task. This task is mainly repetitive, tedious and boring such that robots are more fitted to perform these tasks than humans. It is important to note here that these robots only perform actions that it was programmed for and does not utilizes any form of intelligence. It has no visual inputs and no AI that can allow it to take decisions on the go. However, a lot of research is being done that uses **Reinforcement Learning** so that the robot teaches itself to perform task such as grabbing objects of different sizes and shapes. 

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147762005-5509ccf7-b6ae-49e1-9f5b-dbb0f0fe642b.png" />
</p>
<p align="center">
  Fig. Industrial robots in automobile industry.
</p>

Constituting of mechanical arms, the industrial robots can lift objects ranging from ```1Kg to 400 Kg```. The mechanical arm robots can be found in all sorts of factories but they are mainly suited in metal working and automobile industry for cutting, painting, welding and assembling tasks. At RT Knits, we will use the robot for packaging, palletizing and handling of goods.

Industrial robots differ in various configurations however, they can further be segmented into two main parts: **Serial Kinematic** and **Paralell Kinematic**.

#### 1.1 Serial Kinematic
Serial kinematics consists of an open chain of links similar to the human arm. The should, arm, wrists and fingers all form a chain of links and joints. This configuration is normally used for higher payloads and have a much more flexibility to reach a desired target location with more than one possible orientations. The flexibility is measured w.r.t the number of ```degrees of freedom(dof)```. Our robotic arm has ```6``` dof: ```3``` in position and ```3``` in orientation. Serial kinematics also allows for larger workspaces such that the robot can reach object far from its base. However, their two main disadvantage sare that they are slower and less accurate than the paralell kinematic system. This happends because each link adds some weight and errors to the chain hence, the total error is cummulative of the number of joints. It is possible to build a robot with more joints however, we also increases the complexity to control the entity.


 
<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147844879-0f6a5e08-447b-4cae-a9b6-011f970062e3.png" />
</p>
<p align="center">
Fig. Da Vinci's drawing of human arm(left) and Serial kinematic drawing of robotic arm(right).
</p>

To sum up about Serial Kinematics:
- Higher payloads(5 Kg - 300 Kg)
- Higher number of dof
- Larger workspace
- Slower movements
- Lower accuracy



#### 1.2 Paralell Kinematic
In a paralell manipulator, the actuated joints are not all attached to the ends of the other actuated joints. Instead the actuated joints are placed in paralell such that each joint is connected both to the ```end-effector``` and to the ```ground``` in the manipulator. This configuration is mainly used in setting where the payload to be lifted is lower(```<10Kg```) and the required dof is only ```3``` or ```4```. However, there exists robots which can lift higher payloads and have ```6``` dof such as the ```Gough-Stewart platform``` but these are not common in industrial setting. In contrast to serial kinematics, paralell manipulators have a limited workplace but can move much faster and have a higher accuracy because errors do not add up between joints. 

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147762191-a359c48e-8dab-4cbc-acab-d9ca6fee52ab.png" />
</p>
<p align="center">
  Fig. Figure shows 6 actuated joints connected in paralell. 
</p>

To sum up about paralell kinematic:
- Lower payload(<10 Kg)
- Fewer dof
- Smaller workplace
- Faster movements
- Higher accuracy

The video below shows the distinction between serial and paralell manipulators. 


https://user-images.githubusercontent.com/59663734/147761262-b8dc0e5a-9c0d-48e5-87e8-6bd215ef2ac4.mp4

<p align="center">
Fig 1. ([Video by Oleksandr Stepanenko](https://www.youtube.com/watch?v=3fbmguBgVPA))
</p>

#### 1.3 6-axis Anthropomorphic Robotic Arm
Our 6-axis robot is a ```serial kinematic``` structure with ```6``` links connected together by ```6``` joints. The robot starts with a ```base``` fixed to the ground. It has a motor underneath to allow rotation in the horizontal plane. It has a first axis connected to the base to allow the rotation motion - it is called ```J1``` which  stands for ```Joint 1```. Then we have a second and a third axis called ```J2``` and ```J3```. Note here that ```J1```, ```J2``` and ```J3``` is called the ```arm``` of the robot.

We finish with joints ```J4```, ```J5``` and ```J6``` which forms the ```wrist``` of the robot. It is important to observe here that the rotation of the ```wrist``` does not affect the motion of the ```arm``` however, the inverse is not true. 

We also have a ```Mounting Point(MP)``` which shows where the tool is mounted on the robot and a ```Tool Center Point(TCP)``` which is the very last edge of the robot, including the tool, which is also called the ```End-Effector```. If there is no addtional tool, then ```MP``` = ```TCP```. 

The figure below shows the joints and links of the robot.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147778936-b56e2096-0629-456a-aba6-28b00ed7883e.png" />
</p>
<p align="center">
Fig. Description of 6-axis Anthropomorphic Robotic Arm.
</p>

The ```TCP``` has ```6``` coordinates: ```3``` of them describes its ```position(XYZ)``` in space and the other ```3``` describes its ```orientation(UVW)```. The position of the joints axes and the position of the TCP is related to each other. The process of finding position and orientation of TCP given Joints values is called ```Forward or Direct Kinematics```.  Vice versa, finding our Joints values corresponding to a given TCP position and orientation is called ```Inverse Kinematics```.

### 2. Frames
A key requirement in robotics programming is keeping track of the positions and velocities of objects in space. A frame which is essentially a coordinate system with a specific orientation and position. For example, consider the situation whereby we ask a robot to detecta blue ball in a room. It will be futile if the robot report to us that the blue is 1m from the robot because now we want to know where _in the room_ is that blue ball. Providing this information requires us to know each of the following:

- The position of the blue ball relative to the camera.
- The position and orientation of the camera relative to the base of the robot.
- The position of the robot in the room.

#### 2.1 Geometrical frameworks

We need to develop a framework for representing these relative locations and using that information to translate coordinates from one frame of reference to another. In robotics, we have 4 main geometrical frameworks:

##### 1. Global Coordinate System(GCS)
The Global Coordinate System defines the origin and orientation of our ```world```. If our workspace consists of various robots then it is useful to define a unique global reference system so that everyone understands that same global coordinate. 

The World coordinate frame is indicated with the subscript ```w```. This coordinate frame is fixed at a known location in space and assumed not to move over time.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147844482-e0779f99-da84-4c47-b5fd-5042f5969196.png" />
</p>
<p align="center">
  Fig. Global Coordinate System(GCS) in green.
</p>


##### 2. Machine Coordinate System(MCS)
In a workspace with several robots each machine will have its own local coordinate system(LCS or MCS) which is fundamental in programming each individual robot. However, the other robots will not understand and interact with a point specified in a machine's local coordinate system. If we have only one robot then; ```LCS``` = ```GCS```.

The machine coordinate frame is indicated with the subscript ```r```. We can envision this coordinate frame as being attached to the base of the robot so that the origin of the frame moves as the robot moves. In other words, the robot is always located at ```(0, 0, 0)``` in its own coordinate frame.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147844561-3b7411b3-4add-47de-95cc-9f3bb8b5e9d8.png" />
</p>
<p align="center">
  Fig. Local Coordinate System(LCS) of each robot in blue and GCS in green.
</p>

##### 3. Tool Coordinate System(TCS)
The origin of the Tool Coordinate System is located at the TCP of the robot. If we have no tool attached to the robot, then the TCS is at the Mounting Point(MP) of the robot.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147844661-418aae4a-c2b8-4b9a-ade6-9b9ab6639914.png" />
</p>
<p align="center">
  Fig. Tool Coordinate System(TCS) in yellow and GCS in green.
</p>

##### 4. Workpiece Coordinate System(WCS)
The most important frame for the operator in the Workpiece Coordinate System or Product Coordinate System. This frame is what the operator's sees and considers as the origin of its workplace and all the programmed points and movements refer to this system. We may have several products with which the robot will interact, hence we will have several WCS with their own origin and orientation.


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147844823-b4d78d15-c4ab-4390-8175-0069d4e30b57.png" />
</p>
<p align="center">
  Fig. TCS in yellow, GCS in green, LCS in blue and Workpiece Coordinate System(WCS) in pink.
</p>

Our goal is to understand how to move from one frame to another and how to transform coordinate points across different frames. 

#### 2.2 Frame Operations
Before we dive into frame operations, it is best we establish some conventions that we will use throughout the project. We have a ```left-handed coordinate system``` and a ```right-handed coordinate system.``` In a right-handed coordinate system we determine the direction of the z-axis by aiming the pointer finger of the right hand along the positive x-axis and curling our palm toward the positive y-axis. The thumb will then point in the direction of positive z-axis as shown below. Note that there is no way to rotate these two coordinate systems so that they align. They represent two **incompatible** ways of representing three-dimensional coordinates. This means that whenever we provide coordinates in three dimensions we must specify whether we are using a left-handed or right-handed system.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147845037-82b4c6e2-e0d2-4d5b-bd78-cb39a33ef49a.png" />
</p>
<p align="center">
  Fig. Left and right-handed coordinate systems and Right-hand rule for determining the direction of positive rotations around an axis.
</p>


We also need to establish a convention for describing the direction of a rotation. Again, we will follow a **right-hand rule**. To determine the direction of positive rotation around a particular axis we point the thumb of the right hand along the axis in question. The fingers will then curl around the axis in the direction of positive rotation.

The position and orientation of an object in space is referred to as its ```pose```. Any description of an object's pose must always be made in relation to some to
some coordinate frame. It is possible to translate a point between any two coordinate frames. For example, given the coordinates of the blue ball in the camera coordinate frame, we can determine its coordinates in the robot coordinate frame, and then in the room coordinate frame. In order to accomplish this we need to specify how poses are represented.

Suppose we have a point ```p``` in space and we have two different observers based in frame <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\mathbb{F}_{1}" title="\mathbb{F}_{1}" /> and <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\mathbb{F}_{2}" title="\mathbb{F}_{2}" /> respectively. Both observers are looking at point ```p``` and the first observer will record the coordinates of the point according to its frame as <img src="https://latex.codecogs.com/svg.image?p_{1}" title="p_{1}" /> and consecutively the second observer records point ```p``` as <img src="https://latex.codecogs.com/svg.image?p_{2}" title="p_{2}" />. Now, the question is if we know the coordinate of point ```p``` in the frame <img src="https://latex.codecogs.com/svg.image?F_{1}" title="F_{1}" /> then how do we find the position of that point from the perspective of frame <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\mathbb{F}_{2}" title="\mathbb{F}_{2}" />. If we want to convert <img src="https://latex.codecogs.com/svg.image?p_{1}" title="p_{1}" /> to <img src="https://latex.codecogs.com/svg.image?p_{2}" title="p_{2}" /> then we need to know how the frames <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\mathbb{F}_{1}" title="\mathbb{F}_{1}" /> and <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\mathbb{F}_{2}" title="\mathbb{F}_{2}" /> are related to each other. As such, the three frame operations are:

1. Translation
2. Rotation
3. Translation + Rotation

##### 2.2.1 Translation
Suppose we are based in frame <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\mathbb{F}_{1}" title="\mathbb{F}_{1}" /> and we observe our point ```p``` with coordinates <img src="https://latex.codecogs.com/svg.image?(x_{},&space;y_{},&space;z_{})" title="(x_{}, y_{}, z_{})" />. Its position from this perspective is <img src="https://latex.codecogs.com/svg.image?p_{1}" title="p_{1}" /> with coordinates <img src="https://latex.codecogs.com/svg.image?(x_{1},&space;y_{1},&space;z_{1})" title="(x_{1}, y_{1}, z_{1})" />. We then move our based to a new frame <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\mathbb{F}_{2}" title="\mathbb{F}_{2}" /> which is translated by an offset <img src="https://latex.codecogs.com/svg.image?\Delta&space;f" title="\Delta f" />. That same point ```p``` when observed from that new frame <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\mathbb{F}_{2}" title="\mathbb{F}_{2}" /> has a new position called <img src="https://latex.codecogs.com/svg.image?p_{2}" title="p_{2}" /> with coordinates <img src="https://latex.codecogs.com/svg.image?(x_{2},&space;y_{2},&space;z_{2})" title="(x_{2}, y_{2}, z_{2})" />. Translation is a ```linear``` operation therefore, when coordinate frames are separated by a pure translation transforming points between frames is straightforward: we only need to ```add``` or ```subtract``` the three coordinate offsets.Hence, the new position of ```p``` is simply: 

<p align="center">
  <img src= "https://latex.codecogs.com/svg.image?p_{2}&space;=&space;\Delta&space;f&space;&plus;&space;p_{1}" title="p_{2} = \Delta f + p_{1}" />
</p>

Breaking up for each coordinates, we get:

<p align="center">
  <img src= "https://latex.codecogs.com/svg.image?\begin{pmatrix}&space;x_{2}\\&space;y_{2}\\z_{2}\\\end{pmatrix}&space;=&space;\begin{pmatrix}&space;x_{1}\\&space;y_{1}\\z_{1}\\\end{pmatrix}&plus;\begin{pmatrix}&space;\Delta&space;x_{}\\&space;\Delta&space;y_{}\\\Delta&space;z_{}\\\end{pmatrix}&space;" title="\begin{pmatrix} x_{2}\\ y_{2}\\z_{2}\\\end{pmatrix} = \begin{pmatrix} x_{1}\\ y_{1}\\z_{1}\\\end{pmatrix}+\begin{pmatrix} \Delta x_{}\\ \Delta y_{}\\\Delta z_{}\\\end{pmatrix} " />
</p>


The offset <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\Delta&space;f&space;" title="\Delta f " /> expresses how the old frame <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\mathbb{F}_{1}" title="\mathbb{F}_{1}" /> is seen from the new frame <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\mathbb{F}_{2}" title="\mathbb{F}_{2}" />. In the example below, we have a negative offset along the y-axis. 

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147848752-85e11c3f-b642-4d88-b6df-f8fae882093d.png" />
</p>


##### 2.2.2 Rotation
To find out how the position of our ```p``` changes when rotating our base frame, we need to pre-multiply the position coordinates with a ```rotation matrix```. The content of the matrix depends around which axis we rotate and the angle of rotation <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\Theta&space;" title="\Theta " />. Below shows the rotation matrices when rotating around each individual axes:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147849401-60088e51-75c8-4a09-bad8-1b6b80095604.png" />
</p>


- **X-axis:**

![CodeCogsEqn (10)](https://user-images.githubusercontent.com/59663734/147849126-7b825033-60fa-471f-9616-6a36ca1566a6.png)

- **Y-axis:**

![CodeCogsEqn (7)](https://user-images.githubusercontent.com/59663734/147849071-6a33388a-4177-4c41-9126-4a35f6244413.png)

- **Z-axis:**

![CodeCogsEqn (8)](https://user-images.githubusercontent.com/59663734/147849090-b3ce363d-b961-456c-a8e7-666f5483920e.png)

In all cases, when the angle of rotation is ```0``` degrees, the rotation matrix reduces to the ```Identity matrix```. That is the diagonal of the matrix becomes **one** and the rest becomes **zero** and that is the reason we have **cosine** along the diagonal and **sines** outside of it.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147849232-70e58184-7823-4347-8546-36ebd0ed5523.png" />
</p>


Furthermore, when we ```tranpose``` the rotation matrix we find the same matrix but with an opposite rotation angle, i.e, the rotation in the opposite angle. 

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147849221-8e315da5-02f7-4b56-9bdf-b3abaa2f307d.png" />
</p>


##### 2.2.3 Translation + Rotation







### 3. Forward Kinematics


### 4. Inverse Kinematics


### 5. Path Planning

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147780861-025a3448-f5f5-4020-85ba-5f74f85d218f.png" />
</p>


## Implementation

## References

## Conclusion

