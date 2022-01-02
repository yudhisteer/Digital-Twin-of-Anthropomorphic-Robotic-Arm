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
Specifying orientation in three dimensions is more complicated than specifying position. There are several approaches, each with its own strengths and weaknesses.

###### Euler Angles
In Euler angles orientation is expressed as a sequence of three rotations around the three coordinate axes. These three rotations are traditionally referred to as ```roll```, ```pitch``` and ```yaw```. When working with Euler angles it is necessary to specify the order that the rotations will be applied: a 90o rotation around the x-axis followed by a 90o rotation around the y-axis does **not** result in the same orientaion as the same rotations applied in the opposite order. There are, in fact, twelve valid rotation orderings: ```xyz, yzx, zxy, xzy, zyx, yxz, zxz, xyx, yzy, zyz, xzx, and yxy```.

As convention, each rotation is performed around the axes of a coordinate frame aligned with the earlier rotations which is referred as ```relative``` or ```intrinsic``` rotations.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147849707-cbcc2f4f-6cf0-4760-bc7d-d9af51d54e18.png" />
</p>


###### Rotation Matrix

To find out how the position of our ```p``` changes when rotating our base frame, we need to pre-multiply the position coordinates with a ```rotation matrix```. The content of the matrix depends around which axis we rotate and the angle of rotation <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\Theta&space;" title="\Theta " />. Below shows the rotation matrices when rotating around each individual axes:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147849465-f2b943cf-864e-48b5-ae1c-b31c14703b62.png" />
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

It is possible to represent **any** orienation as a product of three rotation matrices around the x, y and z axes. It is straightforward to convert from an Euler angle representation to the corresponding rotation matrix. For Euler angles represented using relative rotations, the order is reversed.That is, a 45° roration about the x-axis, followed by 30° about the -axis and a 75° about the z-axis is represented as <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;R_{relative}&space;=&space;R_{z}(75^{\circ&space;})&space;\cdot&space;&space;R_{z}(30^{\circ&space;})\cdot&space;&space;R_{z}(45^{\circ&space;})&space;" title="R_{relative} = R_{z}(75^{\circ }) \cdot R_{z}(30^{\circ })\cdot R_{z}(45^{\circ }) " />

In general, rotation of robots are usually given in ```A```,```B``` and ```C``` angles arounf the ```X```,```Y``` and ```Z``` coordinate system. That is we first rotate of an angle A around the x-axis, then an angle B about the y-axis and finally an angle C about the z-axis. The notation is referred as ```Improper Euler angles```, or ```RPY angles``` or ```Cardan angles```.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147851360-dab3f768-ae58-4b2f-b996-2bac78d78ad1.png" />
</p>

Two important properties of rotation matrices are that:

1. Any orientation can be achieved composing Euler angles which is relatively easy to perform.
2. Any rotation matrix can be decomposed in Euler angles although that is is not that straightforward.

The rotation matrix then becomes:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147851708-712b25b9-a40e-4b6f-8da0-db42713a3d1f.png" />
</p>

Note that when decomposing that matrix to find the angle A,B and C we can get two results for an angle. That is, we can reach the same global rotation with different individual rotations around the base axes. Euler angles are not unique! Combined with the choice of axis orderings, there are ```24``` possible conventions for specifying Euler angles.

When <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;B&space;=&space;\pm&space;90^{\circ&space;}" title="B = \pm 90^{\circ }" />, <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;cosB&space;=&space;0" title="cosB = 0" />. The matrix is simplied but now A and C are no longer ```linearly independent```. We are facing a ```singularity``` which is a partivcular configuration whreby an ```infinite``` number of solutions exist. We can only compute the sum and difference of A and C but not their **individual** values.

Some important properties of the rotation matrix: 

- The rotation matrix is an ```orthogonal``` matrix, i.e, its transpose is equal to its inverse. This is important for us and calculating the inverse of the matrix can be difficult whereas transposing it is eaxy and quick.


<p align="center">
  <img src= "https://latex.codecogs.com/png.image?\dpi{110}&space;R^{T}&space;=&space;R^{-1}" title="R^{T} = R^{-1}" />
</p>

- The ```determinant``` of the matrix is equal to ```1```, a property which is useful when checking if the rotation matrix has the correct input. If the determinant is not ```1``` then it is not a correct rotational matrix however, that does not mean that any matrix with determinant ```1``` is a rotational matrix. 

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147852029-b6f2100e-2836-42b9-a531-1dd484519690.png" />
</p>


- The product of the matrixes are ```associative``` but not ```commutative```. This means that rotating about x first and then z is not the same thing as rotating about z first and then x.

To sum up:

Euler angles are relatively easy to visualize, but they are inconvenient to work with from a mathematical point of view. The key problem is that the mapping from spatial orientations to Euler angles is discontinuous: small changes in orientation may cause big jumps in the required representation. This can cause difficulties when we need to smoothly update the orientation of a moving object over time. A possible solution to this would be to use ```Quaternions```. As a general rule in robotics, we use Euler angles only for the interface of the operator(**Visualization**) but we use Quaternions for all internal calculations(**Interpolation**). 



##### 2.2.3 Translation + Rotation(Homogeneous Transformation)
Now we need to put ```translation``` and ```rotation``` together into an individual transformation. The Homogeneous transformation matrix ```T```, combines the rotation matrix ```R```, for example around the Z-axis, but can be any generic rotation around the XYZ frame, and then the translational offset Δ, which can also be any generic translation along the XYZ frame.  The combination of the two is a ```4x4``` matrix, filled with zeros under the rotation matrix and with a ```1``` under the translational offset.  Combining the two operations together means that with a **single** matrix multiplication we can perform both translation and rotation of our frame at the **same time**.


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147852474-eb838ccf-6c84-4ea6-8860-0db1bc100f5d.png" />
</p>

It is important to remember that the order is **not** ```commutative```: ```we first rotate, and then translate```.  If we would first translate and then rotate, the result would be totally different.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147852785-21601287-d34e-47dd-9f0b-52b7980300a1.png" />
</p>

If we have a point <img src="https://latex.codecogs.com/svg.image?p_{0}" title="p_{0}" /> in one frame and want to find out its coordinates as seen from a different frame, we simply pre-multiply by the homogenous matrix.  Note that because the matrix is a 4x4 and the point only has three coordinates, we need to pad its vector with a ```1``` at the end. The final expression for <img src="https://latex.codecogs.com/svg.image?p_{1}" title="p_{1}" /> looks like that. If we only have a translation and no rotation, the cosine elements along the diagonal of the rotation part will all be ```1s``` and the elements outside the diagonal will be ```0s```, so that we fall back to a simple linear offset addition for the translational effects. Similarly, if the translation offset is zero, then we only have the rotation elements left.

**Example:**

Imagine we have a very simple mechanical structure with only one rotating axis. Note that the position of the TCP indicated by the white dot keeps changing following the rotating axis. Now we want to find the position of the TCP in the fixed robot **base frame**. For that we need some geometrical parameters: the length of the arm ```l```, and its height ```h```. The position of the TCP as seen from the **mobile frame** of the robot’s moving link is actually constant. We call it <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{0}" title="p_{0}" /> with coordinates  <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\begin{bmatrix}&space;l&&space;&space;0&&space;&space;0&&space;&space;1\\\end{bmatrix}^{T}" title="\begin{bmatrix} l& 0& 0& 1\\\end{bmatrix}^{T}" />.



<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147853494-719e41da-834a-4f45-a7ee-4bfccf994f5f.png" />
</p>

Now, to find out <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{1}" title="p_{1}" /> from <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{0}" title="p_{0}" />, we need to find out what transformation modifies the old mobile frame into the new fixed frame at the base. We first have a rotation by an angle θ around the Z-axis, and then a translation of ```h``` along Z. Note that both are positive as seen from the new frame. So we can write down the homogenous transformation using an offset ```h``` along the Z-axis and a rotation θ around the same Z axis. ```h``` is always constant, but the angle θ can change all the time because it is driven by the rotating axis. So we pre-multiply <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{0}" title="p_{0}" /> by the matrix ```T``` to find <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{1}" title="p_{1}" />.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147853434-6c5a2d73-70a8-4507-a6d8-da8884a9e6f0.png" />
</p>

To sum up:

- If we have the position of a point given in one frame and want to find its position from a different frame we need to apply some transformations.

- The first and simplest is a ```linear translation```, which mathematically is a simple ```addition```of an ```offset```.

- If the frames are rotated with respect to each other, we need to use a ```rotation matrix```, which is built as a combination of rotations around ```individual``` axes.

- Finally, if both ```translation and rotations``` are present, we can use a ```homogeneous matrix```, which combines both operations into a ```single``` one.


### 3. Forward Kinematics
After studying frame operations, we can now build a geometrical model that will allow us to calculate the position and orientation of each link. This model is knows as a ```Kinematic``` model. There are two types of kinematic model: ```forward transformation(or Direct transformation)``` and ```inverse transformation```.

Forward kinematics is defined as the process of calculating the position and orientation of the **TCP** given the current values of the **joint axes**. We know the position of the joints from the values obtained from the encoders of the electric servo motors. Using these values, we need to calculate the position ```XYZ``` and orientation ```UVW``` of the **TCP** always w.r.t the **base frame**. This function is a simple transformation between the base frame and the TCP frame.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147854128-9f4f0357-5502-47f6-9894-fb9114724ce9.png" />
</p>

The final transformation matrix is a homogeneous transformation that depends on all the joints positions and mechanical parameters of the robot. What we do is simply compose a ```chain``` of frames translations and rotations from the **base** to the **TCP**.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147866651-9563d079-2275-41ee-80f2-9fd89308cee3.png" />
</p>

The final point <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{1}" title="p_{1}" /> can be expressed in the base frame as its local coordinates <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{0}" title="p_{0}" /> pre-multiplied by the homogeneous frame transformation. Note that now all those frames can rotate and translate with respect to each other, according to the joints values, so their angles are not constant, but ```parameterized```.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147866892-b01e8799-8981-4c58-8c6e-e1b73a1962a8.png" />
</p>


So what are these parameters of our kinematic model? That is, what translations and rotations we observe between the different frames?

- we have all the ```rotations``` introduces by the movements of the joints: from joint ```1``` to ```6```. These are **not constant** in time and change accordingly to the motors’ movements.

- we have all the ```translational``` effects due to the mechanical size of the arm. The distance from each joint to the next as shown in the picture above, from the base to the first joint, all the way to the last one. Unlike the rotational parameters, the translational parameters are actually **constant**, because the mechanical size of the arm does not change in time.


#### 3.1 Forward Kinematics: Base to TCP
We will now build our kinematic model step by step from the base to the TCP.

##### 3.1.1 Base to J1
We have a translation and a rotation along the ```Z-axis```. The homogeneous transformation, <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;T_{1}" title="T_{1}" />,  for this first step is built from the rotation matrix <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;R_{1}" title="R_{1}" /> and the offset <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;a1z" title="a1z" />.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147867607-5ace8f7c-e7fd-4328-a1ce-5d0196e434d1.png" />
</p>

- Translation: 
<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147867559-2e01cd06-2eb7-46ed-9a8b-59869df9d4d3.png" />
</p>


- Rotation J1 around Z: 
 
<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147867412-1dbd97c7-e4ab-4dc4-9eac-e54e2721c9b5.png" />
</p>

- Homogeneous Equation: 

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147867461-ece65697-ae73-4ccb-8141-544ecd4b7568.png" />
</p>

##### 3.1.2 J1 to J2
The second step goes from the first to the second joint. Here we have two translations, one along ```X``` and one along ```Z```. Plus the rotation of the second joint around the ```Y axis```. We build the homogeneous transformation matrix <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;T_{2}" title="T_{2}" /> accordingly.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868001-e8ac72b0-87eb-4fb6-be24-c410812202cd.png" />
</p>


- Translation:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147867839-1fea42f0-0133-432d-a7ad-ed053ed5f6f9.png" />
</p>

- Rotation J2 around Y:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147867869-c73d22f6-20a9-4e87-bba5-6188adc798c2.png" />
</p>

- Homogeneous Equation:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147867911-0e7a9e5a-2360-4004-94e7-36bd0f439cbe.png" />
</p>



##### 3.1.3 J2 to J3
From J2 to J3 we have a vertical translation along the ```Z axis```: ```a3z```. This is usually the longest and heaviest arm of the robot and the motor at joint 2 needs to be quite large. We build the homogeneous transformation matrix <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;T_{3}" title="T_{3}" /> using the translation along the ```Z-axis``` and then the rotation around the ```Y-axis```.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868018-117c5f01-6943-4183-adb4-c5cc4e19af86.png" />
</p>

- Translation:
<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868108-3e63ca8a-ea34-4c81-997f-71806e7e4895.png" />
</p>


- Rotation J3 around Y:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868096-1494e952-c96c-43d8-8e6a-303d4b8452e3.png" />
</p>



- Homogeneous Equation:
<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868125-f855c5d2-00a4-4d57-9a9f-c9ec2ee1ec2e.png" />
</p>




##### 3.1.4 J3 to J4
The actual mechanical configuration can change according to the robot we are working on. Here,  we have only two offsets, one along ```Z``` and another one along ```X```. In other robots, there might also be one offset along ```Y```. The homogeneous equation is build using the translation ```a4x``` and ```a4z``` and the rotation matrix about ```X```.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868378-5595cc90-9c8f-4f37-b2fa-542da69c4f9d.png" />
</p>


- Translation:
<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868262-21ff0786-80a8-458a-bb23-c76d62f29488.png" />
</p>

- Rotation J4 around X:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868247-46697002-6ed3-452b-b6b3-fc7db526c549.png" />
</p>

- Homogeneous Equation:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868285-27992a53-802d-4907-85d4-5c6c13ee4a25.png" />
</p>


##### 3.1.5 J4 to J5
With the next step we reach the middle of the wrist called the ```wrist center point```. We simply add a translation along the ```X axis``` and a rotation around ```Y```.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868660-7e5b6ede-ff1a-4a72-8c63-3d221183efa4.png" />
</p>


- Translation:
<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868584-82c47897-19b5-4374-b86c-afdb546e22de.png" />
</p>

- Rotation J5 around Y:
<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868597-111edef6-0910-4342-8c06-a1962b793cc9.png" />
</p>


- Homogeneous Equation:
<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868611-c0d9bf6d-f3c8-4862-b910-8d404be5019a.png" />
</p>


##### 3.1.6 J5 to J6
We finally reach the end of the robot with the last step: one additional translation and rotation along ```X```.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868720-88f0594e-0e61-45f7-bd69-911bceec22e9.png" />
</p>


- Translation:
<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868745-11c43b6b-760f-4e6c-9b39-f0e868083fb6.png" />
</p>


- Rotation J6 around X:
<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868768-1766f708-992a-4e76-a626-a36727550487.png" />
</p>


- Homogeneous Equation:
<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868790-8f87d70f-943e-47b4-b446-5e40d035331b.png" />
</p>


##### 3.1.7 Base to J6
The ```XYZ``` coordinates of the TCP as seen from the base frame can be obtained by pre-multiplying the position of the TCP in its local frame, which is ```zero```, the **origin** of the TCP frame, by all the homogeneous transformations(T1 to T6) we obtained for each frame in the previous six steps. The advantage of this solution is that it can be applied to any open kinematic chain.

For the rotation coordinates ```ABC``` of the TCP we do the same thing, using the rotation matrices we built in the previous steps. Since the product is **associative**, we can split the calculations in two:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147869004-3a708874-9e10-4954-a2f1-7f8d2cc2f2e0.png" />
</p>


1. The ```first```, ```second``` and ```third``` joints determine the rotation of the ```arm```, from the ```base``` to the ```wrist center point```.
2. Then the ```fourth```, ```fifth``` and ```sixth``` joints determine the rotation of the ```wrist``` from the ```wrist center point``` to the ```TCP```.

- Position:
<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868868-1ebfb50a-71fd-4255-b85b-c58a17bd5a54.png" />
</p>


- Rotation:
<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147868917-ff4a5103-374e-4d91-a5aa-c5f874c74d42.png" />
</p>

##### 3.1.8 GCS to Base to J6
So far we have calculated the position and orientation of the TCP as seen from the local robot’s frame or Machine Coordinate System(MCS). Now we want to know the position of the TCP in the Global Coordinate System(GCS) so that other robots can also work with it.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147875836-48bdbeeb-62d3-40d1-b471-ecc56f19abb3.png" />
</p>


For the position, we need to pre-multiply by the homogeneous matrix that describes the transformation between local machine coordinate(MCS) system and global world coordinate system(GCS).


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147875781-eacdc275-addb-4d74-83e2-4d8c734a2c0b.png" />
</p>

For the orientation, it is enough to pre-multiply by the rotation matrix.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147875763-4f914146-39b9-43ad-8182-39f0eb631159.png" />
</p>



##### 3.1.9 Base to J6 to TCP
Now if the TCP is not the mounting point but instead we have a tool at the end of our MP then we need to add the homogeneous matrix and the rotation matrix after the multiplication chain.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147876108-568ed5fe-9c9c-488e-95d5-26de1eb2e1a3.png" />
</p>

- Position:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147876154-e170c4f0-97f1-4ec2-9508-293439e531e7.png" />
</p>


- Rotation:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147876168-3c6cef73-d7fd-4255-aa57-cb234a3632ab.png" />
</p>


#### 3.2 Mechanical Coupling
In practical, there are axes of the robot that are internally mechanically coupled. For example, a movement of J5 causes a movement of the J6 even if the motor of J6 is not moving. Hence, we need to compensate for this effect in our calculation. 

In the picture below, only J1 is moved and J2 is remained fixed. However, due to the mechanical coupling, J2 is forced to move even if the motor was not activated. The white dotted lines shows the difference between the actual positions reached by the arm **with** and **without** mechanical coupling. That difference is a function of the ```coupling coefficient``` between J1 and J2.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147877147-a5e7a391-fac8-4a39-bbd6-358985ff7a49.png" />
</p>

We normally solve the direct transformations between joints and TCP with the forward kinematic function. But the joints positions are what we read from the encoders, they do not represent the actual position of the joints angles if an additional mechanical coupling is present.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147877334-bc2d3a70-dbcb-41ee-9bec-4ceb2f7364ed.png" />
</p>


For our 6-axis robot, we introduce ```coupling coefficients``` in the forward kinematic model so that the actual position of the TCP is calculated correctly. Before we send those joints values from the encoders to the forward kinematics, we need to adjust them by the coupling coefficient: for example the actual position of J5 and J6 might be influenced by the position of the J4. In that case the final value is what the motor sees plus the coupling factor function of J4.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147877578-587dd073-7ea5-4edf-9de0-20249b9e809c.png" />
</p>


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147877401-c5ccfd67-13aa-4f79-84c1-77ec4a1ff0e3.png" />
</p>


### 4. Inverse Kinematics
Up until now, we have studied forward kinematic whereby we are given the joint angles(J1 to J6) of the axes and we need to calculate the position and orientation of the TCP or End-Effector. We showed how we could use the Homogeneous transformation matrix H, to find the location of our end-effector.

In inverse kinematics we are given the TCP location and we are asked to find the joints angles(J1 to J6) that result in that particular TCP location. Inverse kinematics is more useful for us as this is the kind of situation that we will be dealing with in real-world. That is, we may know a particular object location that our end-effector will need to grab and we will then need to find out the desired joints angles of our robots to achieve this task of grabbing the object. Remember that the TCP coordinates are expressed relative to the base frame.


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147880223-c8870731-ebd5-4e06-a318-5ffc00480b75.png" />
</p>

Robots are programmed by the operator in ```path space coordinates (XYZ)```, which are clearly understood. But motion control software acts directly on the motors, which move the joints of robots. So we need a way to transform back from ```path space``` to the ```joints space```. 

Unfortunately, the inverse kinematics problem is so ```hard``` that sometimes it is even ```impossible``` to solve the problem in closed-form, that is we cannot write down equations to derive the joints axes values and a numerical approach is required. In other times, we will need a couple of ```assumptions``` and even ```geometrical intuition``` in order to solve the inverse kinematics.


For a 6-axes robot the condition that allows for a closed-form solution is the ```configuration of the wrist```. We have two kinds of wrists: ```spherical``` and ```non-spherical``` wrist robots.

#### 4.1 Spherical wrist robot
In the spherical wrist, the three rotating axes ```J4,J5,J6``` all intersect at ```one``` point at the ```wrist center point```. This configuration is the most common in the industry and allows for a nice closed-form solution.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147880700-1f97bbdb-e5cc-48f1-8649-d546d0d2c6b6.png" />
</p>


#### 4.2 Non-spherical wrist robot
In the non-spherical wrist, the rotation axes do **not** intersect at one point. This configuration requires a ```numerical solution```(trial-and-error procedures - approximation to the solution of a mathematical equation) so it is mathematically more complex and computationally more expensive. However, it does **not** suffer from ```singularity``` issues and can always perform movements at ```constant speed```. For that reason it is used often in ```painting``` applications, where a constant speed is strictly required to spray a uniform layer of paint.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147880705-397009aa-7c93-4d03-8fe7-57c25e085be2.png" />
</p>

#### 4.3 Non-unique solutions
Another problem that makes solving inverse kinematics complicated, is the fact that, given a particular target position and orientation of the TCP, there might be more than one way for the joint axes to configure in order to reach that pose. In other words, the solution to the inverse kinematics is not always unique!

#### 4.3.1 Non-unique solution I: Up and Down
In this example we have a target position which is same for both robot. Both robots reached the ```same``` target with two ```different``` configurations. Observe that the TCP position and orientation are absolutely the ```same``` in both cases. But the joint axes configurations are ```different```!

We call the two options ```UP``` and ```DOWN```. This is also a problem for the operator while programming the robot. In our software interface we need to provide a way for the operator to choose the preferred option, either up or down, or maybe we can automatically select the solution closest to the current joints configuration.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147884034-624297c0-f748-4621-a6f8-b40e20675548.png" />
</p>


#### 4.3.2 Non-unique solution II: Front and Back


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147884680-89e3f4f8-4951-4e46-be42-d3588d8fb8ce.png" />
</p>



#### 4.3.3 Non-unique solution III: Positive and Negative


#### 4.4 Singularities




### 5. Path Planning

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147780861-025a3448-f5f5-4020-85ba-5f74f85d218f.png" />
</p>


## Implementation

## References

## Conclusion

