letter_learning_interaction
===========================

This package contains a set of nodes useful for creating a user interaction around the premise of a Nao learning handwriting.

Tested with ROS Hydro and Indigo.

Provided nodes:
---------------
- `learning_words_nao.py`: the main node for managing the CoWriter interaction. Controls the robot's speech and head movements to facilitate the interaction (prompting the user for inputs including feedback and words to write); manages the learning algorithm state; sends requests for shapes to be written and responds to received user feedback on said shapes. [requires the [shape_learning library](https://github.com/chili-epfl/shape_learning), python naoqi SDK and a running `display_manager_server` node]

- `display_manager_server.py`: provides services which allow for access to a `ShapeDisplayManager`'s methods by other ROS nodes. I.e., allows multiple nodes to position shapes on the display (as needed by a shape learning algorithm) and request which shapes are present at a particular location (as needed to process user feedback on shapes), etc.

- `gesture_manager.py`: listens for generic gesture occurances and translates them into shape-specific gestures based on the location at which they occurred. [requires a running `display_manager_server` node]

- `word_card_detector.py`: listens for frames which represent fiducial markers for a dictionary of words, and publishes the associated words (used to request a word to be written by the user). Tested with [chilitags for ROS](https://github.com/chili-epfl/ros_markers).

  
