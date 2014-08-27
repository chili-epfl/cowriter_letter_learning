#cowriter_letter_learning: Teaching a robot to write

A set of ROS nodes which facilitate the user interaction allowing a robot to be taught handwriting.

![Photo of CoWriter interaction](https://github.com/chili-epfl/cowriter_letter_learning/raw/master/doc/cowriter_demo.jpg)

*An example result of the interaction achievable with cowriter_letter_learning: Words are requested by showing cards to the robot with [chilitags](https://github.com/chili-epfl/chilitags) on them; children correct the robot's [simulated handwriting](https://github.com/chili-epfl/nao_writing) by providing demonstrations on a tablet, which are used to update the robot's [shape learning algorithm](https://github.com/chili-epfl/shape_learning).*

##Usage
####With a webots simulated Nao running:

(With the `shape_learner` app deployed on the tablet, a webcam plugged in, and [word cards with fiducial markers](https://github.com/chili-epfl/cowriter_letter_learning/raw/master/doc/tags5-9_wordgame_robotWriting.pdf) printed)

```
roslaunch letter_learning_interaction nao_learning.launch
```

*(Alternatively, `rostopic pub /words_to_write std_msgs/String "use" -1` may be used to send words to write (e.g. 'use') manually, without detecting cards.)*

####With a [ROS-enabled Nao](https://github.com/ros-nao/nao_robot):
(With the `shape_learner` app deployed on the tablet and [word cards with fiducial markers](https://github.com/chili-epfl/cowriter_letter_learning/raw/master/doc/tags5-9_wordgame_robotWriting.pdf) printed (robot's camera will be used))

Install chrony on the computer so that the robot may sync its clock.

On the robot:

```
sudo /etc/init.d/ntpd stop
sudo ntpdate (computer's IP)
sudo /etc/init.d/ntpd start
```

`ntpdate -q (computer's IP)` should then give ~0 offset, indicating that the clocks are synchronised.

```
export ROS_MASTER_URI=http://(computer's IP):11311
roslaunch nao_bringup nao.launch
```

On the computer acting as the ROS master:

```
roslaunch letter_learning_interaction nao_learning.launch use_sim_nao:=false nao_ip:=(nao's IP) use_external_camera:=false
```

Dependencies
------------
- [shape_learning](https://github.com/chili-epfl/shape_learning) for the letter learning algorithm,
- [nao_writing](https://github.com/chili-epfl/nao_writing) for the synchronised trajectory tracing capabilities for the robot,
- [ROS for Android](https://github.com/rosjava/rosjava_core) for the tablet app,
- (optional) [ros_markers](https://github.com/chili-epfl/ros_markers) for detecting the word cards,
- (optional) [gscam](https://github.com/ros-drivers/gscam) for using an external webcam to detect word cards.

For more information, see the readmes in internal directories and the parameters which are available for specification in the launch files.

