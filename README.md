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

Robots are programmed by the operator in ```path space coordinates (XYZ)```, which are clearly understood. But motion control software acts directly on the motors, which move the joints of robots. So we need a way to transform back from ```path space``` to the ```joints space```. More explanation about these two spaces are found below:

> An industrial robot can be controlled in two spaces: ```Joints space``` and ```Cartesian space```. Hence, there are two sets of position-mode motion commands that make an industrial robot move. For joint-space motion commands (sometimes incorrectly called ```point-to-point commands```), you simply specify — directly or indirectly — a ```desired set of joint positions```, and the robot moves by translating or rotating each joint to the desired joint position, simultaneously and in a linear fashion. For Cartesian-space motion commands, you specify a ```desired pose for the end-effector``` AND a ```desired Cartesian path``` (linear or circular). To find the necessary joint positions along the desired Cartesian path, the robot controller must calculate the ```inverse position``` and ```velocity kinematics``` of the robot. Singularities arise when this calculation fails (for example, when you have division by zero) and must therefore be avoided.

> Try jogging a six-axis robot arm in ```Joint space```, and the only time the robot will stop is when a joint hits a ```limit``` or when there is a ```mechanical interference```. In contrast, try jogging the same robot in ```Cartesian space``` and the robot will frequently stop and refuse to go in certain directions, although it seems to be far from what you think is the workspace boundary. **A robot singularity is a configuration in which the robot end-effector becomes blocked in certain directions.**

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

##### 4.3.1 Non-unique solution I: Up and Down
In this example we have a target position which is same for both robot. Both robots reached the ```same``` target with two ```different``` configurations. Observe that the TCP position and orientation are absolutely the ```same``` in both cases. But the joint axes configurations are ```different```!

We call the two options ```UP``` and ```DOWN```. This is also a problem for the operator while programming the robot. In our software interface we need to provide a way for the operator to choose the preferred option, either up or down, or maybe we can automatically select the solution closest to the current joints configuration.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147884034-624297c0-f748-4621-a6f8-b40e20675548.png" />
</p>


##### 4.3.2 Non-unique solution II: Front and Back
The next example is even more extreme. We move the two robots to the same target TCP pose and we notice how they select completely different solutions whereby the second motion is absolutely redundant and overly complex. 

The main difference here is J1, which can either point in the front or the back of the robot, hence, the name ```FRONT``` and ```BACK```.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147884680-89e3f4f8-4951-4e46-be42-d3588d8fb8ce.png" />
</p>



##### 4.3.3 Non-unique solution III: Positive and Negative
The next example is more subtle. The final configurations look the same but they are not. While J1, J2 and J3 are in the same position and give the same pose to the arm, the last three joints are in different positions and give a different configuration to the wrist. Notice that the J5 has opposite values in the two cases, hence, we call one ```POSITIVE``` and the other ```NEGATIVE```.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147900767-8d008fbb-a1ba-493b-912e-cc1e0b6c4cfe.png" />
</p>

In all these previous examples we were able to find ```two distinct``` solutions to the inverse kinematics problem.


#### 4.4 Singularities

Now, we will see that there are cases where an ```infinite``` number of solutions are available. These critical points are called ```singularities```, and occur for some specific target poses of the TCP. There are ```3``` types of singularities: ```Shoulder Singularity```, ```Wrist Singularity``` and ```Elbow Singularity```.


##### 4.4.1 Shoulder Singularity
In the shoulder singularity, the wrist center point from J4, J5 and J6 aligns with the center point of J1. If we try to move from this position we will notice that the J1 and J6 will start to rotate very fast in opposite direction. The TCP will remain almost still causing a very inefficient robot motion. We can find a fixed solution for most joints, except for J1 and J6. These two are ```linearly dependent``` and we can only find their ```sum``` or ```difference```. Therefore, an ```infinite``` number of possible solutions are available.

Recall that a matrix is singular when its ```determinant``` is ```zero```. The associated linear system has an infinite number of solutions because two or more variables are dependent on each other.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147901490-851b8ca2-cd27-4011-9c90-2510d5d45342.png" />
</p>




##### 4.4.2 Wrist Singularity
In the wrist singularity, J5 is ```0```, while J4 and J6 are **not** independent anymore. We can move them around without modifying the final pose of the TCP. We have an ```infinite``` number of solutions. Normally, we manually pick a value for one of the two joints, for example J4, and then the other joint is automatically fixed.


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147903862-5c59a62e-ffd7-46db-8f73-0a776b22f9c3.png" />
</p>



##### 4.4.3 Elbow Singularity
Imn the elbow singularity, J2, J3 and the wrist center point aligns together. When these 3 points are aligned, the elbow of the robotics arm will get stuck in the so
called elbow singularity no matter where the TCP is. Elbow singularities look like the robot has “reached too far”, causing the elbow to **lock** in position.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147903645-515e143c-cb5e-41ea-b053-38665ff88385.png" />
</p>

To sum up:

- the inverse kinematic problem is more complex than forward kinematics, not only because it is difficult to find a solution, but because sometimes there are many, or even infinite possible solutions we need to pick from.
- the definition of singularity is when the robot has an ```infinite``` number of kinematic possibilities to reach the same end position - an infinite number of ways to get to the ```same``` spot.
- what a robot's kinetic algorithm is trying to do is solve a ```complex equation``` of where to put all our axes(J1 to J6) to get our tool where we need it to be. The operator will just drag this robot somewhere and say record and we just take for granted that it gets there. But if there are an ```infinite``` number of solutions to get there then it does not know which one to take hence, we find singularity and the robot ```faults out```.

Below is a video showing the singularities explained above:



https://user-images.githubusercontent.com/59663734/148587446-15e434bb-f6a5-40fc-8f08-8ef141b521ad.mp4
<p align="center">
  [Video by Mecademic Robotics](https://www.youtube.com/watch?v=lD2HQcxeNoA)
</p>


##### 4.4.4 Avoiding Singularities
It is kind of hard to tell you where we are likely to find the most singularities since they are not at a given position, they are ```relative``` to the robot configuration. In simpler words, singularities occur when a robot loses a degree of freedom. 

There are two ways we can find these singularities:

**- Inspection:** Here we just observe and determine what would be a singulafrity for that robot.

**- Calculation:** We write the Forward Kinematic equations, take the derivative of the velocity, find the Jacobian matrix and set its determinant equal to zero to find the joints angles that would make the determinant = 0.

Singularities most occur in nature when we place objects to be grabbed by the TCP at right angles and having the object below at the base of the robot. The image below shows that J5 is 2 degrees away from zero. The table is too low and the object is too close to the base of the robot. The robot would go pick the object but if we go a little bit back then we hit a singularity. There are many places singularities can occur becasue anytime J5 is ```0``` I can hit a singularity. One important thing to remember is that when in a situation as depicted in the image below, the programmer is powerless. There is nothing robotics programmers can do as they are bounded by unsolvable mathematical equations. However, we can redesign the workplace to avoid such occurances as explain below.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147918132-70959e4b-beb8-4b31-875d-674c90772583.png" />
</p>



Below are ```6``` ways to avoid singularities:

**1. Choose another robot:** We can ```only``` get a singularity with a 6-axis or more robot. If we have a four axis robot, there is no such thing as singularity. It cannot happen because any place we want to go there are no two axes fighting each other hence, we have only one solution.

**2. Singularity Avoidance package:** This is a package when  buying a robot that imbed some built-in intelligence to the robot to avoid these singularities. It's going to make movements that will result in only one solution.

**3. Re-design workplace:** We could relocate the robot by lowering it relative to the picking surface or change the table's postion, i.e, we increased the height and placed it a bit further from the base and this result in a change in the articulation of the robot arm. We now have a nice bend between J4 and J5(57.6°).


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147919682-f606283d-d09e-4546-8b42-60b06b13ac96.png" />
</p>

**4. Offsetting the tool:** By offsetting the tool as shown below, we force the robot to be further away from the object. In that way we induce a small angle(about 15 degrees) whereby we used to have only about a 2 degrees angle. Offsetting the bracker allows us to gain some reach - our J6 can not swing - and it also allow us to manipulate the tool as per our needs.


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147928638-8458dfb4-4af3-4c2c-8074-ab24d81754f1.png" />
</p>

**5. Pivot the tool:** We can also pivot our tool such that when picking the object, the tool is perpendicular to the object but we have about 30 degrees angle at our J5. We are far away from zero therefore far away from a singularity. 


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147929587-71af6b68-cb86-4cb8-be64-528fdc74ecd5.png" />
</p>

**6. Tilt robot:** The next solution would be to tilt the robot - about 30 degrees - such that the robot is fixed on a wedge block and we have a nice J4 and J5 interaction with a bend of 24 degrees. 

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147931517-ed134db6-831c-44b2-8054-ea9c92a04108.png" />
</p>

To sum up:

- it is mostly the mechanical engineer work to find a suitable solution for the robot to avoid singularities. The programmer should already plan this on his software. 
- it is important to plan ahead of the position of the robot or workbench or tool so as to avoid redundant changes in the physcial robot.
-  robot singularities can only be avoided by properly designing our robotic cell (and that includes the design of the adaptor plate for our end-effector).


#### 4.5 Inverse Kinematics: Arm
Earlier we discussed that our robot is a ```spherical wrist``` robot as ```J4```,```J5``` and ```J6``` all intersect at the ```wrist center point(WP)```. That point is crucial because it is dependent only on ```J1```, ```J2``` and ```J3```. That is, only a movement of J1, J2 and J3 can change the position of WP and not J4, J5 and J6. Hence, we can split the robot into two parts: ```arm``` and ```wrist```. We will process as follows:

1. Given the position of the WP, we can find J1, J2 and J3.
2. Then gicen the position and orientation of the TCP, we find J4, J5 and J6.

This process is called ```decoupling```, i.e, we decouple the wrist from the arm and this process is only possible for spherical wrist and J4, J5 and J6 do not affect the position of WP.

#### 4.5.1 Finding WP

We have shown before that the difference between WP and TCP is ```a6x``` which is the offset between J5 and J6. However, this offset runs along the x-axis of the local TCP frame, which we call <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\hat{x}" title="\hat{x}" /> in the robot’s base frame.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148023273-6a59a4de-4b8d-48ef-8741-3d2693c9df2d.png" />
</p>


1. Given the orientation of the TCP in Euler angles, we know how to compose the equivalent ```rotation matrix``` by multiplying together all the individual rotations around the XYZ axes.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147943639-d0a19751-92fe-4a66-a4e8-cf18febed4d8.png" />
</p>


2. We then take the rotation matrix ```R```, multiply it for the local ```x``` vector <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\begin{bmatrix}&space;1&&space;&space;0&&space;&space;0\\\end{bmatrix}^{T}" title="\begin{bmatrix} 1& 0& 0\\\end{bmatrix}^{T}" /> and we find how this vector is seen from the base.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147943664-cd7eecff-3d0f-4c38-876d-ba65f1b78899.png" />
</p>

3. From the multiplication above, we get the first column of R, which is <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\hat{x}" title="\hat{x}" />. We subtract ```a6x``` times <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\hat{x}" title="\hat{x}" /> from the ```TCP``` position to find out the ```WP``` position.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147943691-0a92b8de-f086-49a6-b442-f01a3c93efe7.png" />
</p>

After finding WP, we can now find J1, J2 and J3.

#### 4.5.2 Finding J1
The position of J1 **only** affects the X(<img src="https://latex.codecogs.com/png.image?\dpi{110}&space;WP_{x}" title="WP_{x}" />) and Y(<img src="https://latex.codecogs.com/png.image?\dpi{110}&space;WP_{y}" title="WP_{y}" />) coordinates of the wrist point, not the vertical position Z(<img src="https://latex.codecogs.com/png.image?\dpi{110}&space;WP_{Z}" title="WP_{Z}" />). So we can reduce the problem to a ```2-dimensional``` planar problem and observe that J1 is the angle between the X and Y coordinates of WP. The solution is simply to take the ```arctangent```.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147945551-7654be09-05e2-4c49-b03a-a7e128c7c359.png" />
</p>

Recall that two solutions are possible for this problem, depending on the pose that we select: either ```front``` or ```back```. In the first case J1 is what the arctangent function returns; in the second case we need to add 180 degrees.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147946136-0e0941ae-5482-4fd9-93e1-9f6bbc7a0779.png" />
</p>

Finally, if both X and Y are zero, the wrist point is straight above the base origin and we have a ```shoulder singularity```. Any position for J1 would be a correct solution. We usually fix J1 equal to the actual position of the joint.

#### 4.5.3 Finding J2 and J3
When J1 is fixed, movements of J2 and J3 only happen on a ```vertical``` plane. So we can simplify the problem by removing one dimension and reduce the workspace to a ```2D``` geometry as shown below. We need to focus on the green triangle to find J2 and J3. 

- First using pythagoras theorem, we unified the X and Y coordinates into a single XY(<img src="https://latex.codecogs.com/png.image?\dpi{100}&space;WP_{xy}" title="WP_{xy}" />) component.
- We will focus on the green traingle to solve for J2 and J3. One of its side is <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;\rho&space;" title="\rho " /> and its horizontal and vertical projections are ```h``` and ```l``` respectively. Since we know the mechanical size of the robot we can easily find ```h``` and ```l```. We then using pythagoras theroem to find <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;\rho" title="\rho" />.
- We already know another side of the green triangle: ```a3z```. We know need the last side which is ```b4z```. We observe that it is the hypothenus of the yellow triangle on top with legs ```a4z``` and ```a4x+a5x```.
- In order to avoid a situation whereby the robotic arm tries to reach an object toor far from the TCP, we need to apply the ```triangle inequality```. We limit the value of <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;\rho" title="\rho" /> using this rule.

>  The sum of the lengths of any two sides of a triangle has to be greater than the length of the third side and the difference of the lengths of any two sides of a triangle has to be less than the length of the third side.

- Now we need to find <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;\alpha&space;" title="\alpha " /> using the arctan of ```h``` and ```l```.
- We find <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;\beta&space;" title="\beta " /> using the cosine rule.
- We observe that J2 + <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;\alpha&space;" title="\alpha " /> + <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;\beta&space;" title="\beta " /> = 90°. We solve for J2. Note that it could also be plus <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;\beta&space;" title="\beta " />, if we were to choose the ```DOWN``` pose instead, with the second link pointing downwards and the third upwards.
- We find <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;\delta" title="\delta" /> using arctan.
- For the top yellow traingle, we observe that J3 + <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;\gamma&space;" title="\gamma " /> + <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;\delta&space;" title="\delta " /> is 180°. We solve for J3.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147964345-ac0893e5-2bee-4c82-96eb-b2cb2f4188af.png" />
</p>


#### 4.6 Inverse Kinematics: Wrist
The first three joints(J1, J2 and J3) determine the position of the wrist point and also the orientation of the arm from the base up to the wrist center.
We call that rotation matrix <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;R_{arm}" title="R_{arm}" /> The rotation of the TCP is given by the matrix ```R```, which we already found composing the ```ABC``` Euler angles. The missing rotation in between those two is given by the <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;R_{wrist}" title="R_{wrist}" /> matrix.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148021469-91bc6df3-9534-409c-8119-d7fe1cec5b74.png" />
</p>


- ```R``` is the product of <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;R_{arm}" title="R_{arm}" /> times <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;R_{wrist}" title="R_{wrist}" />, we can solve for <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;R_{wrist}" title="R_{wrist}" /> pre-multiplying both sides by <img src="https://latex.codecogs.com/svg.image?R_{arm}^{-1}" title="R_{arm}^{-1}" /> and simplifying the right side. <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;R_{wrist}" title="R_{wrist}" /> is then the product between <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;R_{arm}" title="R_{arm}" /> transpose and ```R```.
- Since <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;R_{arm}" title="R_{arm}" /> is an orthogonal matrix, we know that its inverse is equal to its transpose. 
- Remember that is <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;R_{arm}" title="R_{arm}" /> the rotation matrix from the ```base``` to the frame created by ```J1```, ```J2``` and ```J3```. Hence, we need to compose the rotations of the joint ```J1``` around the ```Z-axis```, and of ```J2``` and ```J3``` around the ```Y-axis```.
- The ```R``` matrix is found from ```A,B,C``` hence we can now derive <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;R_{wrist}" title="R_{wrist}" />.
- After finding <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;R_{wrist}" title="R_{wrist}" />, we need to decompose it into its individual rotations of ```J4```, ```J5``` and ```J6```. Remember that ```J4``` rotates around ```X```, ```J5``` around ```Y``` and ```J6``` around ```X``` again.
- We then need to decompose the <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;R_{wrist}" title="R_{wrist}" /> matrix into Euler angles to find J4, J5 and J6.
- However, if ```cosJ5 = 1``` then ```J5 = 0```. We will then have a ```wrist singularity```. There is ```no``` rotation around the ```Y-axis```, only two consecutive rotations around ```X```. There is no way to separate ```J4``` and ```J6``` anymore. Only the ```sum``` of their values is known, because it is essentially a ```single``` rotation about the ```X-axis```. The <img src="https://latex.codecogs.com/png.image?\dpi{100}&space;R_{wrist}" title="R_{wrist}" /> ,matrix then becomes:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148022371-2bb4bcdb-e78e-49bc-9a10-47976b9d9d6b.png" />
</p>

- The solution is to fix one of the two angles, for example ```J4``` equal to its current value, and find ```J6``` accordingly.
- We always try to minimize the robot’s movement’s distances to reach a target position. Hence, since angles are periodic we can add or subtract <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;2\pi&space;" title="2\pi " /> to get the most convenient solution.

We must remember to adjust for our coupling joints. The output of the inverse kinematics model is the positipm of the real joints. They must be adjusted to calculate the correct motor target positions. 


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148023927-0bafe79a-548e-4f3b-abb1-36f419466848.png" />
</p>


To sum up:

- Manipulator tasks are normally formulated in terms of the desired position and orientation.
- A systematic closed-form solution applicable to robots in general is not available.
- Unique solutions are rare, multiple solutions do exist.
- Inverse kinematics: Given a desired posiion X,Y,Z and orientation A,B,C, we find values of J1,J2,J3,J4,J5 and J6.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148035299-8fed7aa0-f91d-4e89-b8f5-905eb5d69941.png" />
</p>



### 5. Path Planning
In path planning, we will study the geometrical description of the robot’s movement in space. The geometrical curve to move between points is what we call the ```path```. If we have to points in space - ```A``` and ```B``` - we can have several ways to travel from A to B. We can have the shortest route which is the displacement - a straight line from A to B or we could have some random curve path joining the two points. Once we fix the geometry we want, we also need to generate a ```time-dependent trajectory``` to cover the path.

- The easiest way to move from one point to another one, is by simply moving the joints axes of the robot to their target positions. This is called ```point-to-point(PTP)``` movement. PTPs are **fast** but they do not provide any specific **control** over the geometrical path of the TCP.
- If we want control over the geometry of the path, then we need an ```interpolated movement```, for example a ```line```. Other interpolated movements are the ```circle```, and the ```spline``` which is a smooth interpolation through some target points in space.
 
<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/147780861-025a3448-f5f5-4020-85ba-5f74f85d218f.png" />
</p>

#### 5.1 Point-to-point(PTP)
Point-to-point is a linear interpolations of the joint axes of the robot. How does it work? Suppose we start from point <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{0}" title="p_{0}" />, where all the joints have certain values and we want to reach point <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{1}" title="p_{1}" />, where the joints have different values. We simply ```linearly``` **increase** or **decrease** the joint angles, from their starting to their target values. 

- We use the parameter ```t``` to describe the equation of this movement. 
- ```t = 0``` at the starting point and ```t = 1``` at the end of the movement.
- If we pick any value between ```0``` and ```1```, the joint axes have a value of <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p&space;=&space;p_{0}(1-t)&plus;p_{1}t" title="p = p_{0}(1-t)+p_{1}t" />.
- when ```t = 0```, <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p=p_{0}" title="p=p_{0}" /> and when ```t = 1```, <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p=p_{1}" title="p=p_{1}" />.


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148075320-06ae0f3a-0d6a-4839-9039-6010c749de2b.png" />
</p>


**Note:** 

1. ```t``` is not ```time```. ```t``` is just a parameter describing the curve from ```0``` to ```1```. It describes the ```path``` but not the ```trajectory```. Here, we only plan the ```geometry```, not the ```speed``` of the movement. So no time is involved.
2. This linear interpolation happens in the ```Joint``` space. The joint angles move linearly from their starting position to their target value. For example, joint ```J1``` could move from ```0``` to ```80``` degrees. However, this linear movement of the ```joints``` does **not** translate at all into a linear movement of the ```TCP```. If we imagine the first joint moving from ```0``` to ```80``` degrees: the TCP actually moves around a ```circle``` in space(remember that J1 is a rotation around the Z-axis.)!
3. While PTP movements are very simple to program and fast to execute, they also have **no** control over the actual path of the end effector of the robot. At planning time we do **not** know what the ```position``` and ```orientation``` of the TCP will look like.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148078598-aa691caf-fafc-400b-9cf5-211fd3dd9a38.png" />
</p>

Although the TCP movement is not a line, the formula we need to calculate this movement is a linear interpolation between <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{0}" title="p_{0}" /> and <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{1}" title="p_{1}" />. As ```t``` increases, <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p\mapsto&space;p_{1}" title="p\mapsto p_{1}" />. As shown from the graphs above, the value of ```J1``` changes linearly as ```t``` increases.


#### 5.1.1 Point-to-point(PTP) vs Path Interpolated
- If we want to reconfigure the arm, e.g. change from ```UP``` to ```DOWN```, the only way we can do that is with a ```PTP``` movement, where the joint axes are ```free``` to move ```independently``` from each other to their final target value.

- A ```path-interpolated``` movement does ```not``` allow that. If we start a line in space with an ```UP``` configuration we will arrive at the end of the line with the same ```UP``` configuration. That is because the ```geometry``` of the arm is ```restricted``` during path movements. It **cannot** stretch and reconfigure itself in a different way without sending the TCP out of its planned path.

- ```PTP``` movements, on the other hand, do **not** care about the TCP path and can essentially do whatever they want to the geometry of the links.

To sum up, ```PTP``` movements are ```time-optimal```, that is, they always take the ```shortest``` time possible to complete from start to end. Finally, and most importantly, a PTP movement is the only movement that allows a modification of the **actual** joint configuration.


#### 5.2 Path Interpolation
In order to describe a target pose for our TCP in space, we need ```6``` coordinates hence, the reason for our robot to be a ```6dof``` robot. Suppose we have a starting point <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{0}" title="p_{0}" /> with coordinates ```XYZ``` and we need to move to our next position <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{1}" title="p_{1}" /> with coordinates ```X'Y'Z'```. This time we are expressing the joints in the ```Path``` space instead of the ```Joint``` space. Moving from one point to the next can be done linearly, i.e, one unit top and one unit to the right. And to return back we have two options; either one unit down and one unit left or one unit left and then one unit down. Either way we will return to our starting position! This is because we are working in a ```Euclidean``` space, where vectors add up linearly. The process can be seen below:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148589773-f5fc7ed7-b9a4-4dfb-a08c-696e1d64387c.png" />
</p>

However, this is not the case for our TCP angles ```UVW``` because ```Euler``` angles do not interpolate linearly. This happens because Euler angles do suffer from [Gimbal lock](https://www.youtube.com/watch?v=zc8b2Jo7mno) where the order in which we apply the three rotations affects later rotations in the sequence. 
Below shows the process of first rotating around X of 90 degrees, then rotate around Z also of 90 degrees to reach <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{1}" title="p_{1}" />.  And then we go back, X of -90 and Z of -90. If rotations did add up linearly like positions we would go back to the initial pose but this not what happened!

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148594837-0bfa9b4f-70fa-40d7-b782-ff991559d325.png" />
</p>

A better way to interpolate these orientations is to convert our Euler angles into another representation before interpolating them. ```Rotation matrices``` is one solution but matrices are not suitable for interpolations as they cannot be parameterized with one single variable running from 0 to 1. A better solution would be to use ```Quaternions```.




#### 5.3 Quaternions
Since Euler angles suffer from singularities at 90 degrees, we need another way in order to interpolate angles: Quaternions. The general expression for quaternion is:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148680674-1ed54918-ce9d-4cb0-b7bf-a593c1e17e00.png" />
</p>

where <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;xi&plus;yj&plus;zk" title="xi+yj+zk" /> is a vector and ```w``` is a scalar. i,j and k are complex numbers hence: 

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148680770-f27c2f6d-02af-4a90-962d-4220c02df79e.png" />
</p>

The ```4``` elements ```x,y,z,w``` define the quaternion and are the number of elements it needs to describe an orientation, as opposed to ```3``` elements for Euler angles and ```9``` for a rotation matrix. We typically work with ```unit``` quaternions: that means a ```normalized``` form, where the norm of the quaternion is simply the Euclidean size of the vector of elements x,y,z,w.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148680853-53634d73-e514-49c5-8aed-35c784856470.png" />
</p>

where <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;{\left\|&space;q\right\|}&space;=&space;\sqrt{x^{2}&plus;y^{2}&plus;z^{2}&plus;w^{2}}" title="{\left\| q\right\|} = \sqrt{x^{2}+y^{2}+z^{2}+w^{2}}" />.

Suppose we have a vector ```v``` in 3-dimensional space and we want to rotate it about point ```p``` at an angle <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\theta&space;" title="\theta " />.


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148681110-83782090-0414-4964-b552-fe55b59139b8.png" />
</p>


We will use quaternion to rotate it and we start by defining our quaternion and its complement:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148681217-1a189ea2-0319-402c-aa37-ef35bed894cb.png" />
</p>

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148681244-597ed690-2aca-4d35-af8f-fa0233cafce9.png" />
</p>

If we are rotating at an angle of <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;\theta&space;" title="\theta " />, then ```a,b,c,d``` are defined as :

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148681324-4daa00da-f4e4-4792-b30d-17ae1674c28e.png" />
</p>


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148681345-55f563db-6571-4560-a650-b54db8f0a656.png" />
</p>


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148681352-42e25222-e6d3-4a24-a562-23bbd3f727b1.png" />
</p>


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148681378-c04b8ed7-10bd-4ac2-896c-46b9c2c837cf.png" />
</p>


Any given orientation is described by one ```unique``` quaternion. That is unlike Euler angles, with which many representations were possible for the same orientation in space. After defining ```a,b,c,d```, our corresponding unit quaternion is defined by:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148681495-babae7e5-530b-4a5e-ac11-3e51fffde938.png" />
</p>

If ```p``` is the point around which we are rotating, we write ```p``` as a quaternion:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148681542-1353143e-aabb-470d-89d1-f961854dd6af.png" />
</p>

And finally the rotation is :

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148681579-a9d54829-206d-47af-a500-9090b55458d3.png" />
</p>


#### 5.3.1 Quaternions Advantages over Euler Angles
- Compared to Euler angles, quaternions do not suffer from ```singularities```: they offer a ```unique``` undisputable way to identify rotations.
- They also offer an easy way to interpolate ```linearly``` between orientations.

#### 5.3.2 Quaternions Advantages over Rotation Matrix

- Quaternions are more compact: using only ```4``` elements instead of ```9```.
- Matrices are also impossible to ```interpolate```, and they are not numerically stable.
- It is possible that a rotation matrix undergoes several computations and comes out ```non-orthogonal``` because of numerical approximations, that is, its determinant is not ```1``` anymore. In that case the matrix is very difficult to recover.

#### 5.3.3 Spherical Linear Interpolation(Slerp)
Given two orientations, represented by the two quaternions <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;q_{1}" title="q_{1}" /> and <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;q_{2}" title="q_{2}" /> what is the shortest and simplest way to go from one point to the other? Since we move along a sphere, there are infinite ways to do this but we are interested in the optimal path - the ```torque-optimal``` solution which uses the minimum energy to travel from the start to the end. 

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148687212-37bcd0b2-92b6-4310-8667-16cbb852ab6f.png" />
</p>


The solution is defined by the formula below, which is equivalent to a linear interpolation in Euclidean space, but this time we move on a ```sphere```. We still use the parameter ```t```, which goes from ```0``` to ```1```.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148687135-bdac03af-42e7-4322-9cb5-9da94be6e19e.png" />
</p>

We can travel along the rotation at ```constant angular speed```. This kind of interpolation is called ```Slerp```. Alpha is the angle between the two quaternions, and it is calculated the same way as the angle between vectors on plane:


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148687346-6bdac9ab-7a73-4b37-a18c-a068c23104f6.png" />
</p>

Note that the problem with the ```arccosine``` is that it can return two different angles between the quaternions: either the shortest one of the opposite one. To guarantee that we always use the shortest angle we need to make sure that the dot product between the two quaternions is ```positive``` before taking the cos inverse. If we happen to have a negative dot product then we should invert it by inverting one of the two quaternions, i.e, multiple one of the quartenion by -1. We will still reach the same rotation, just using a different path. 

Also, if the angle between the two orientations is very small we could face numerical instabilities with that <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;sin\alpha&space;" title="sin\alpha " /> at the denominator. Observe that a small arc on the sphere essentially degrades into a ```linear``` segment, so we can use the standard linear interpolation instead, and have more stable calculations.


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148687708-5ab10ccb-6c0d-42ff-b094-ba5db760406c.png" />
</p>

As shown below, when moving the TCP around a particular axis, only the orientation of the TCP chnages and not its position, i.e, the XYZ values remain constant.

https://user-images.githubusercontent.com/59663734/148687992-9160776e-9002-4c84-a01b-000af5f90433.mp4

#### 5.3.4 Implementation
We have the ```start```(<img src="https://latex.codecogs.com/png.image?\dpi{110}&space;A_{0}B_{0}C_{0}" title="A_{0}B_{0}C_{0}" />) orientation of the robot and the user provides a ```target```(<img src="https://latex.codecogs.com/png.image?\dpi{110}&space;A_{1}B_{1}C_{1}" title="A_{1}B_{1}C_{1}" />) orientation. We cannot interpolate the ABC Euler angles directly so we interpolate quaternions using ```Slerp```. 

1. We first transform the ```Euler``` angles into ```Rotation``` matrices and from there into ```Quaternions```. 
2. Then we apply ```SLERP``` to interpolate between <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;q_{0}" title="q_{0}" /> and <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;q_{1}" title="q_{1}" />
3. But because we control the motors of the robot we need to solve the ```inverse``` kinematics to find out the values of the ```joint``` axes. So we need to transform ```q``` back into a ```Rotation``` matrix and then ```Euler``` angles as input to our inverse kinematic function.


<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148688596-c342ffe3-4a53-4885-8d5b-37da2e0b673b.png" />
</p>


To sum up:

- There are a few advantages of using quaternions over Euler angles or rotation matrices, the most important of all for our purposes being the possibility of interpolate them. When we move the robot from one pose to another we need to change its orientation from the initial one to the final one. This smooth change over time is called ```interpolation```. We cannot interpolate matrices or Euler angles directly. Quaternions can do that easily with ```SLERP```.



#### 5.4 Lines
The simplest interpolation in the path space is a ```line```. We start from point <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{0}" title="p_{0}" /> with coordinates <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;(x_{0},y_{0},z_{0})" title="(x_{0},y_{0},z_{0})" /> and want to reach <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{1}" title="p_{1}" /> with coordinates <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;(x_{1},y_{1},z_{1})" title="(x_{1},y_{1},z_{1})" />. The formula for linear interpolations is as explained before: <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p=p_{0}(1-t)&plus;p_{1}t" title="p=p_{0}(1-t)+p_{1}t" />, with ```t``` going from ```0``` to ```1```.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148692202-b8a3e7d1-ce9c-4ce8-958d-a63269907bef.png" />
</p>

We may have two situation in practice:

1. In the first case the starting and target points have the same orientations, so the interpolation only happens on the position coordinates ```XYZ```. 
2. In the second case the initial and final orientations are different. So, while the position coordinates interpolate ```linearly```, the orientation interpolates using ```SLERP```. The two interpolations happen in ```parallel``` at the same time.


#### 5.5 Circle
Two points are not enough to define a circle in space. We need three points: the starting point <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{0}" title="p_{0}" />, an intermediate point <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{1}" title="p_{1}" /> and the final target point <img src="https://latex.codecogs.com/png.image?\dpi{110}&space;p_{2}" title="p_{2}" />. The parametric formula for the position of a point describing a circle in space is the following:

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148695447-19695319-a181-4fea-a01c-10592fee01a4.png" />
</p>

- C: center point


![CodeCogsEqn - 2022-01-09T223840 083](https://user-images.githubusercontent.com/59663734/148695874-9f158368-1f24-4591-972f-d42268bee319.png)

- r: radius


![CodeCogsEqn - 2022-01-09T223656 556](https://user-images.githubusercontent.com/59663734/148695792-86e77768-1674-4f51-b264-229e94fe8bf9.png)

- U: normalized vector from the center to the starting point

- V: a vector perpendicular to U, lying on the circle plane
- t: the parameter that runs around the circle, from zero to the ending angle

Note: We need to check the points are not ```collinear```, i.e, no points lie on the same line in space. The vectors U and V are perpendicular to each other and form the analogue of the X, Y axes on a plane.

<p align="center">
  <img src= "https://user-images.githubusercontent.com/59663734/148696550-4f59bfcb-0aba-4d92-ac47-0b2a8f09c4ba.png" />
</p>


So starting from ```t = 0``` we point along the vector ```U``` and we then rotate around using the formula in ```2D```: ```rcos(t) + rsin(t)```. This is a group of three individual equations: one in ```x```, one in ```y```, and one in ```z```. We are only focusing on position here, because the orientation will follow the ```SLERP``` interpolation: from the ```start``` orientation to the ```middle``` orientation and to the ```ending``` one.


## Implementation

## References
1. https://www.youtube.com/watch?v=L7J_9OSxGvA
2. https://www.youtube.com/watch?v=BPjqH_e5y3s
3. https://robohub.org/3-types-of-robot-singularities-and-how-to-avoid-them/
4. https://www.mecademic.com/en/what-are-singularities-in-a-six-axis-robot-arm
5. https://www.youtube.com/watch?v=vCEWORZbD3Y
6. https://www.youtube.com/watch?v=1zTDmiDjDOA
7. https://www.youtube.com/watch?v=unwUt3kkgvE

## Conclusion

