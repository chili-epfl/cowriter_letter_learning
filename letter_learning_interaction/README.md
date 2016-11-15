letter_learning_interaction
===========================

This package contains a set of nodes useful for creating a user interaction around the premise of a Nao learning handwriting.

Tested with ROS Hydro and Indigo.

Provided nodes:
---------------
- `learning_words_nao.py`: the main node for managing the CoWriter interaction. Controls the robot's speech and head movements to facilitate the interaction (prompting the user for inputs including feedback and words to write); manages the learning algorithm state; sends requests for shapes to be written and responds to received user feedback on said shapes. [requires the [shape_learning library](https://github.com/chili-epfl/shape_learning), python naoqi SDK and a running `display_manager_server` node]

- `display_manager_server.py`: provides services which allow for access to a `ShapeDisplayManager`'s methods by other ROS nodes. I.e., allows multiple nodes to position shapes on the display (as needed by a shape learning algorithm) and request which shapes are present at a particular location (as needed to process user feedback on shapes), etc.

- `tablet_input_interpreter.py`: listens for tablet inputs from the user and translates them into shape-specific events based on the location at which they occurred. [requires a running `display_manager_server` node]

- `word_card_detector.py`: listens for frames which represent fiducial markers for a dictionary of words, and publishes the associated words (used to request a word to be written by the user). Tested with [chilitags for ROS](https://github.com/chili-epfl/ros_markers).

Letters dataset configuration
-----------------------------

`learning_words_nao.py` requires one training dataset per letter. You can learn
more about these datasets (format, where to get them, how to create them...) in
the [`shape_learning` project](https://github.com/chili-epfl/shape_learning).

The launch argument `letter_model_dataset_directory` must points to a directory
containing files (or symlink to files) named `[a-zA-Z].dat`, one per letter
(note that these files are loaded 'on-demand', so if you know you won't be using
a certain range of letter, you do not need the corresponding datasets).

For instance, for a specific experiment, you may have a directory
`/home/nao/datasets/expe1` containing the `.dat` file, and you would call
`nao_learning.launch` this way:

```
$ roslaunch letter_learning_interaction nao_learning.launch letter_model_dataset_directory:=/home/nao/datasets/expe1 [...other options]
```

