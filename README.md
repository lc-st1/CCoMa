# CCoMa
### CWRU Soft Machines and Electronics Lab

This is the repository for the **C**able-actuated **Co**ntinuum **Ma**nipulators (**CCoMa**) extension package for Pybullet.
This software is part of a soft robotics project in the Case Western Reserve Univeristy Soft Machines and Electronics Laboratory. `https://www.caogroup.org/`.

Start Guide
-------
After cloning, move the python file `CCoMa.py` to the project directory. Full PyPI package support will be added in the future.

To use CCoMa in a project, add one of the following line to the python script:

```python
import CCoMa
```
or 

```python
from CCoMa import Cable, CableSet, ContinuumManipulator, generateURDF
```



Documentation
-------
Please see the documentation files `CCoMaGuide.pdf` or `CCoMaGuide.md` for full class, function, and implementation documentation. 


Requirements
-------
This package requires Pybullet >3.2.x as well as the latest versions of Numpy and Math. This is intended for use with Python >3.11.x.


Examples
-------
In the folder labelled `Examples`, there are a few example Pybullet simulations and Continuum Manipulator URDF files. These contain some demos of various parts of the extension, such as generating/loading the custom URDF files, controlling a multi-segmented cable actuated continuum manipulator, and using the actuator classes. These may need to be run from the same folder as `CCoMa.py` in order to import it properly.


Issues
-------
If there are any issues or bugs, please report these on the repository page or talk to Luca Ciampaglia directly at `lsc65@case.edu`.


Future Features
-------
The following features are currently planned for implementation:

-   PyPI Packaging
-   Code Optimization
-   Simulated Servo PID Control
-   Pneumatic Actuator Support
-   Physically-based Sensor Models
-   Real-time FESTO System Control

Any feature requests should either be added as a report or sent to Luca Ciampaglia at `lsc65@case.edu`


License
-------
MIT License

Copyright (c) 2023 Luca Ciampaglia

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

