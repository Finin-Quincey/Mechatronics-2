# Mechatronics II: Working Rovertime

_"This is one small step for hamster... one giant leap for hamsterkind"_

Code for the Mechatronics 2 assignment (Year 4 IDE) to build a robot contained in a rolling 'hamster ball'.

## Specs

- Python 3.8.3 (64-bit conda environment)
- OpenCV 4.4 with ArUco library
- Arduino Nano 33 IoT running compiled Simulink model

## Run configurations

This button always runs whatever file is currently being edited:

[image not found](https://github.com/Finin-Quincey/Mechatronics-2/images/simple_run_button.png)

The problem with that is it sets the Python search path to whatever folder that file is in, so it can't find any code outside of that folder. The better way to run stuff is using the run tab:

[image not found](https://github.com/Finin-Quincey/Mechatronics-2/images/run_tab.png)

In this tab is a little run button next to a dropdown menu, containing _run configurations_:

[image not found](https://github.com/Finin-Quincey/Mechatronics-2/images/run_configurations_dropdown.png)

Run configurations are specified in [`launch.json`](https://github.com/Finin-Quincey/Mechatronics-2/.vscode/launch.json) (open it using the settings icon next to the dropdown). These allow the Python search path to be specified relative to the _workspace_ (the folder this README is in) rather than the file being run.

Finally, you can run whatever was run last using the **Run -> Start Debugging** or by pressing F5:

[image not found](https://github.com/Finin-Quincey/Mechatronics-2/images/run_menu.png)

## Troubleshooting

If Anaconda makes an absolute mess of the install like mine did, the following might help fix it:

https://medium.com/@udiyosovzon/how-to-activate-conda-environment-in-vs-code-ce599497f20d

To make pylint recognise opencv subpackages, add the following to `settings.json` in VSCode:
```
"python.linting.pylintArgs": [ "--generate-members" ]
```

Common importing problems: http://python-notes.curiousefficiency.org/en/latest/python_concepts/import_traps.html

## Useful links

- OpenCV documentation: https://docs.opencv.org/4.4.0/index.html
  - OpenCV Python tutorials: https://docs.opencv.org/4.4.0/d6/d00/tutorial_py_root.html
  - ArUco module reference: https://docs.opencv.org/4.4.0/d9/d6a/group__aruco.html
- Python documentation: https://docs.python.org/3.8/
  - The Python Tutorial: https://docs.python.org/3.8/tutorial/index.html
    - Basics of strings in Python: https://docs.python.org/3.8/tutorial/introduction.html#strings
    - More stringy stuff: https://docs.python.org/3.8/tutorial/inputoutput.html#fancier-output-formatting
    - Control flow in Python: https://docs.python.org/3.8/tutorial/controlflow.html
    - Object-oriented programming in Python: https://docs.python.org/3.8/tutorial/classes.html
  - Python standard library docs: https://docs.python.org/3.8/library/index.html
- Numpy basics: https://numpy.org/doc/stable/user/absolute_beginners.html
