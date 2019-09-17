# HSR-challenge-3
University of Michigan code used for the third HSR Challenge 181213.

Contact: Brent Griffin (griffb at umich dot edu)

## Execution
In background, need to run: <br />
``roslaunch hsr_war_machine amcl.launch``<br />
``bst proxy lambda ./jarvis/index_hsr.js``<br />
``roslaunch rosbridge_server rosbridge_websocket.launch``

Need to have tensorflow sourced to run the challenge script: ``./heimdall.py``.

__Video Demonstration:__ https://youtu.be/sQ_fYik8XKI

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/sQ_fYik8XKI/0.jpg)](https://www.youtube.com/watch?v=sQ_fYik8XKI)

## Setup
The necessary segmentation models (e.g., "r.ckpt") are trained using ``train_osvos_models.py`` at https://github.com/griffbr/VOSVS/tree/master/OSVOS_train. Code is setup for Toyota's Human Support Robot (HSR) using ROS messages, but should be reconfigurable for other robot platforms. Will also need to set up an Amazon Echo or equivalent to convert voice commands to ROS messages.

## Paper
We have a paper detailing our vision and control method used in the challenge: [Video Object Segmentation-based Visual Servo Control and Object Depth Estimation on a Mobile Robot](https://arxiv.org/abs/1903.08336 "arXiv Paper")<br />
[Brent Griffin](https://www.griffb.com), Victoria Florence, and [Jason J. Corso](http://web.eecs.umich.edu/~jjcorso/)<br />
IEEE Winter Conference on Applications of Computer Vision (WACV), 2020

Please cite our paper if you find it useful for your research.
```
@inproceedings{GrFlCoWACV20,
  author = {Griffin, Brent and Florence, Victoria and Corso, Jason J.},
  booktitle = {IEEE Winter Conference on Applications of Computer Vision (WACV)},
  title = {Video Object Segmentation-based Visual Servo Control and Object Depth Estimation on a Mobile Robot},
  year = {2020}
}
```

## Use
This code is available for non-commercial research purposes only.

