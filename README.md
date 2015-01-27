#The CoWriter project: Teaching a robot to write

A set of ROS nodes which facilitate the user interaction allowing a robot to be taught handwriting.

![Photo of CoWriter interaction](https://github.com/chili-epfl/cowriter_letter_learning/raw/master/doc/cowriter_demo.jpg)

*An example result of the interaction achievable with cowriter_letter_learning: Words are requested by showing cards to the robot with [chilitags](https://github.com/chili-epfl/chilitags) on them; children correct the robot's [simulated handwriting](https://github.com/chili-epfl/nao_writing) by providing demonstrations on a tablet, which are used to update the robot's [shape learning algorithm](https://github.com/chili-epfl/shape_learning).*


Dependencies
------------
- [shape_learning](https://github.com/chili-epfl/shape_learning) for the letter learning algorithm,
- [nao_writing](https://github.com/chili-epfl/nao_writing) for the synchronised trajectory tracing capabilities for the robot,
- [ROS for Android](https://github.com/rosjava/rosjava_core) for compiling the tablet app,
- [ros_markers](https://github.com/chili-epfl/ros_markers) for detecting the word cards,
- [gscam](https://github.com/ros-drivers/gscam) for using an external webcam to detect word cards instead of the Nao's camera.

For more information, see the readmes in internal directories and the parameters which are available for specification in the launch files.


##Usage
####With only a tablet
(With the `shape_learner` app deployed on the tablet)

```
export ROS_IP=(computer's IP)
roslaunch letter_learning_interaction nao_learning.launch use_robot_in_interaction:=false
```

`rostopic pub /words_to_write std_msgs/String "use" -1` to send words to write (e.g. 'use') manually.

*Note that console output can be viewed with `rosrun rqt_console rqt_console`.*

![Photo of word learning app progress (initial).](https://github.com/chili-epfl/cowriter_letter_learning/raw/master/doc/cow_initial.png)&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;![Photo of word learning app progress (final).](https://github.com/chili-epfl/cowriter_letter_learning/raw/master/doc/cow_final.png)

*An example of the system learning the word 'cow' (blue) based on user demonstrations with the tablet (green).*

####With a webots simulated Nao running

(With the `shape_learner` app deployed on the tablet, a webcam plugged in, and [word cards with fiducial markers](https://github.com/chili-epfl/cowriter_letter_learning/raw/master/res/tags5-9_wordgame_robotWriting.pdf) printed)

```
export ROS_IP=(computer's IP)
roslaunch letter_learning_interaction nao_learning.launch
```

*(The camera device may be specified by appending `camera_device:=/dev/video1`, for example.)*
*(Alternatively, `rostopic pub /words_to_write std_msgs/String "use" -1` may be used to send words to write (e.g. 'use') manually, without detecting cards.)*
*Note that console output can be viewed with `rosrun rqt_console rqt_console`.*

####With a [ROS-enabled Nao](https://github.com/ros-nao/nao_robot)
(With the `shape_learner` app deployed on the tablet and [word cards with fiducial markers](https://github.com/chili-epfl/cowriter_letter_learning/raw/master/res/tags5-9_wordgame_robotWriting.pdf) printed (robot's camera will be used))

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
export ROS_IP=(computer's IP)
roslaunch letter_learning_interaction nao_learning.launch use_sim_nao:=false nao_ip:=(nao's IP) use_external_camera:=false
```

*Note that console output can be viewed with `rosrun rqt_console rqt_console`.*

Tips
----

- the following command lets you send random words to Nao for writing (change
  `french` accordingly. `cut -c1-4` only keeps the first 4 letters. You can
  change that as well)

```
watch -n15 'rl -c1 /usr/share/dict/french | cut -c1-4 | xargs rostopic pub -1 words_to_write std_msgs/String'
```
