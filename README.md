# Kinematics of an holonomic robot on MATLAB

<a name="table_of_contents"/>

## Table of Contents
1. [Introduction](#introduction_)
2. [Development](#development_)
3. [Conclusion](#conclusion_) 
4. [License](#license_)



<a name="introduction_"/>

## Introduction

A robot is an autonomous system that exists in the physical world, which can sense its environment and act on it to achieve certain goals and act on it to achieve certain goals [Mataric, 2007].

There are two categories of wheeled robots :

* Non-holonomous : most wheeled robots, the robot has physical constraints and therefore must move by following curves.

Examples : Amazon Kiva, HUSQVARNA Automower 420...

![alt text](https://github.com/Clerbout-Francois/Kinematics_holonomic_robot_MATLAB/blob/main/AmazonKIVA.jfif?raw=true)

_Figure 1: Amazon Kiva robot._

* Holonomous : robots using omnidirectional wheels, which can move in a straight line and in all directions. in all directions. Robots in this category have full 360 degree freedom of movement.

Examples : Robotino, KUKA omniMove, KUKA youBot, KUKA KMR iiwa...

![alt text](https://github.com/Clerbout-Francois/Kinematics_holonomic_robot_MATLAB/blob/main/KUKA_Omnimove_Header.jpg?raw=true)

_Figure 2: KUKA omniMove robot._

In the context of the development of industry 4.0, I will focus on the forward and inverse kinematic of a holonomic mobile robot (simulation on MATLAB, programming in C++) in order to design and concept a holonomic robot intended for a future application in logistics. 

As I have already simulate movements of a [robotic arm in 2D](https://github.com/Clerbout-Francois/2D-robot) on MATLAB, I was responsible of the MATLAB simulations (forward kinematics with and without speed vectors and inverse kinematics of the holonomic robot). These simulations will be provided with the project in order to teach students and also to permit them to study the movements of the robot depending on the entry commands before programming on a real holonomous robot.

These simulations are top views of the robot (simplified). You will be asked in the forward_kinematic forward_kinematic_with_speed_vectors programs to enter the speeds of each wheel (positive speed for a clockwise rotating motor and negative speed for a counter-clockwise rotating motor), the movements represented are over 10 seconds and the distances in centimetres. In the last program, you are asked to enter an end point for the robot's centre of gravity.

In this README and this GitHub repository you will find all the three different MATLAB programs I coded. There are many comments in the code (you will find all the comments in English in the file [forward_kinematic](https://github.com/Clerbout-Francois/Kinematics_holonomic_robot_MATLAB/blob/main/forward_kinematic.m), the other codes are commented in French but have the same structure as the first code), if need be do not hesitate to contact me to discuss it or for any help. You can also find a video of the simulation of each program in this repository.

[Table of Contents](#table_of_contents)
<a name="development_"/>

## Development

### Kinematic model and definition of the robot

At the beginning of the design of these different programs, the main difficulty was to determine the kinematic model of this robot. This kinematic model was implemented in the code in the form of a matrix (which you can find on pages 168-169 of this [thesis](https://www.researchgate.net/publication/272673531_Modeling_and_Adaptive_Control_of_an_Omni-Mecanum-Wheeled_Robot)) that I had to modify and arrange in order to make it functional with MATLAB.

This is why, at the beginning of the program, you will find the definition of the coordinates of the corners of the robot as well as its centre of gravity at the origin of the program (i.e. centred on the reference frame) and other points or lengths... All the points and angles defined at the beginning of the program are necessary for the modelling of each robot movement thereafter.

![alt text](https://github.com/Clerbout-Francois/Kinematics_holonomic_robot_MATLAB/blob/main/Program1.png?raw=true)

![alt text](https://github.com/Clerbout-Francois/Kinematics_holonomic_robot_MATLAB/blob/main/Program2.png?raw=true)

_Figure 3: Initialization of variables._

[Table of Contents](#table_of_contents)

***
### Study of the different cases 

As the robot can move in any direction in space, it was necessary to separate its movements into different cases taking into account its angle (its orientation with respect to the abscissa axis). For the orientation of the robot I will work with the trigonometric circle.

![alt text](https://github.com/Clerbout-Francois/Kinematics_holonomic_robot_MATLAB/blob/main/Four-quadrants-circle.jpg?raw=true)

_Figure 4: Trigonometric circle._

I distinguish 2 main cases concerning the rotation speed of the robot (if it is zero or not) and in each of these 2 main cases there are 4 other cases depending on its initial orientation (at the beginning of the movement) if the rotation speed is equal to 0 and only one otherwise.

[Table of Contents](#table_of_contents)
***
**Rotation speed is equal to 0**

In each of the 4 cases, the procedure is the same. That is why I will show you one of them and only the calculations will differ afterwards.

I then start by calculating the coordinates of each endpoint (at the end of the robot's movement).

![alt text](https://github.com/Clerbout-Francois/Kinematics_holonomic_robot_MATLAB/blob/main/Program3.png?raw=true)

_Figure 5: Updating the coordinates of the points._

Then I update very regularly (thanks to a constant step defined previously) the shape of the robot by using set functions, it allows me to keep the shape of the robot and thus to prevent it from deforming during a movement (we keep the same lengths and we keep the perpendicularity between the different sides of the robot).

![alt text](https://github.com/Clerbout-Francois/Kinematics_holonomic_robot_MATLAB/blob/main/Program4.png?raw=true)

_Figure 6: Update of the robot representation._

In this README, I only present the update of the robot representation for an initial orientation between 0 and pi/2, it works the same way for the 3 other cases.

![alt text](https://github.com/Clerbout-Francois/Kinematics_holonomic_robot_MATLAB/blob/main/Program5.png?raw=true)

_Figure 7: Presentation of the case where the robot is oriented between 0 and pi/2 (quadrant I)._


[Table of Contents](#table_of_contents)
***
**Non-zero rotation speed**

In this case, we work as previously presented for a zero rotation speed by calculating the new coordinates of each point and then updating the graphic representation of the robot.

![alt text](https://github.com/Clerbout-Francois/Kinematics_holonomic_robot_MATLAB/blob/main/Program6.png?raw=true)

_Figure 8: Updating the coordinates of the points._

![alt text](https://github.com/Clerbout-Francois/Kinematics_holonomic_robot_MATLAB/blob/main/Program7.png?raw=true)

_Figure 9: Update of the robot representation._


[Table of Contents](#table_of_contents)
<a name="conclusion_"/>

## Conclusion

The idea when starting to code was to produce simple, easy to reproduce (scalable) codes and above all a visual representation accessible to all. I think that the objectives I had set myself have been achieved (maybe if I came back to the project I would make functions that I would call each time to make the program lighter) and that one of the axes of progression would be to use the Robotics System Toolbox of MATLAB for a future project of simulation in robotics. My next simulation project will be done with this toolbox so stay tuned if you are interested !!! :smirk:


[Table of Contents](#table_of_contents)
<a name="license_"/>

## License

Please do not forget that this project is under [MIT license](https://choosealicense.com/licenses/mit/).



[Table of Contents](#table_of_contents)
