# 🪐 Mechatronics II: Working Rovertime

_"This is one small step for hamster... one giant leap for hamsterkind"_

Code for the Mechatronics 2 assignment (Year 4 IDE) to build a robot contained in a rolling 'hamster ball' (team of 4 students). This repository contains the 'mission control' part of the code, which takes images from an overhead camera and processes them to plan a path for the robot and send commands over WiFi accordingly.

![](https://github.com/Finin-Quincey/Mechatronics-2/blob/main/images/overview_labels.png)

## 🛰 Specs

- Python 3.8.3 (64-bit conda environment)
- OpenCV 4.4 with ArUco library
- Arduino Nano 33 IoT running compiled Simulink model
- 720p webcam mounted overhead (2.25m above the floor), looking straight down
- 2.4GHz WiFi communications via UDP

## 🚀 Run configurations

See the [run configurations guide](run_configurations.md).

## 🐍 Troubleshooting

If Anaconda makes an absolute mess of the install like mine did, the following might help fix it:

https://medium.com/@udiyosovzon/how-to-activate-conda-environment-in-vs-code-ce599497f20d

To make pylint recognise opencv subpackages, add the following to `settings.json` in VSCode:
```
"python.linting.pylintArgs": [ "--generate-members" ]
```

Common importing problems: http://python-notes.curiousefficiency.org/en/latest/python_concepts/import_traps.html

## 🔭 Useful links

- [OpenCV documentation](https://docs.opencv.org/4.4.0/index.html)
  - [OpenCV Python tutorials](https://docs.opencv.org/4.4.0/d6/d00/tutorial_py_root.html)
    - [Hough line transform](https://docs.opencv.org/4.4.0/d6/d10/tutorial_py_houghlines.html)
  - [ArUco module reference](https://docs.opencv.org/4.4.0/d9/d6a/group__aruco.html)
  - [Read the docs OpenCV tutorials](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_tutorials.html)
- [Python documentation](https://docs.python.org/3.8/)
  - [The Python Tutorial](https://docs.python.org/3.8/tutorial/index.html)
    - [Basics of strings in Python](https://docs.python.org/3.8/tutorial/introduction.html#strings)
    - [More stringy stuff](https://docs.python.org/3.8/tutorial/inputoutput.html#fancier-output-formatting)
    - [Control flow in Python](https://docs.python.org/3.8/tutorial/controlflow.html)
    - [Object-oriented programming in Python](https://docs.python.org/3.8/tutorial/classes.html)
  - [Python standard library docs](https://docs.python.org/3.8/library/index.html)
- [Numpy basics](https://numpy.org/doc/stable/user/absolute_beginners.html)
