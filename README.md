# Generic 3D Obstacle Detection for AGVs Using Time-of-Flight Cameras

## Abstract

Automated guided vehicles (AGVs) are useful for a variety of transportation tasks.
They usually detect obstacles on the path they are following using 2D laser scanners.
If an AGV should be deployed in a shared space with people, 3D information has to be considered as well to detect unforeseen obstacles.
These can be small objects on the floor or overhanging parts of larger objects, which cannot be seen by the standard 2D safety scanners.
We propose a generic object detection pipeline using 3D time-of-flight cameras, that can be used in real-time on AGVs of low height and demonstrate its robustness to different measurement artefacts.

## Publication

Published in the proceedings of IROS 2016 with the title "Generic 3D Obstacle Detection for AGVs Using Time-of-Flight Cameras":

```latex
@INPROCEEDINGS{buck2016odtof,
  title = {Generic 3D Obstacle Detection for AGVs using Time-of-Flight Cameras},
  author = {Buck, Sebastian and Hanten, Richard and Bohlmann, Karsten and Zell, Andreas},
  booktitle = {Intelligent Robots and Systems (IROS), The International Conference on},
  address = {Daejeon, Korea},
  year = {2016},
  month = oct,
  days = {9-14}
}
```

## CS::APEX (Algorithm Prototyper and EXperimentor for Cognitive Systems)

CS::APEX is a framework based on synchronous dataflow and event-based message passing that
aims to speed up prototyping of new robotic algorithms using visual programming aspects.
For more, visit https://github.com/cogsys-tuebingen/csapex
