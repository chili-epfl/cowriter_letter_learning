cowriter_letter_learning
========================

A set of ROS nodes which facilitate the user interaction which allows a robot to be taught handwriting.

![Photo of CoWriter interaction](https://github.com/chili-epfl/cowriter_letter_learning/raw/master/doc/cowriter_demo.jpg)

*An example result of the interaction achievable with cowriter_letter_learning: Words are requested by showing cards to the robot with [chilitags](https://github.com/chili-epfl/chilitags) on them; children correct the robot's [simulated handwriting](https://github.com/chili-epfl/nao_writing) by providing demonstrations on a tablet, which are used to update the robot's [shape learning algorithm](https://github.com/chili-epfl/shape_learning).*

Example usage:
```
roslaunch letter_learning_interaction nao_learning.launch
```
