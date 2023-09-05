# CCoMa
### CWRU SME Lab    

This is the repository for the **C**able-actuated **Co**ntinuum **Ma**nipulators (**CCoMa**) extension package for Pybullet.


Start Guide
-------
After cloning, move the python file `CCoMa.py` to the project directory. Full package support will be added eventually.

To use CCoMa in a project, add the following line to the python script:

```python
from CCoMa import from CCoMa import Cable, CableSet, ContinuumManipulator
```


Documentation
-------
Please see the documentation file CCoMaGuide.pdf for full class, function, and implementation documentation.

Requirements
-------
This package requires Pybullet 3.2.x as well as the latest versions of Numpy and Math.

Examples
-------
In the folder labelled `Examples`, there are a few example Pybullet simulations and Continuum Manipulator URDF files. These contain some demos of various parts of the extension, such as generating/loading the custom URDF files, controlling a multi-segmented cable actuated continuum manipulator, and using the actuator classes. These may need to be run from the same folder as `CCoMa.py` in order to import it properly.

Issues
-------
If there are any issues or bugs, please report these on the repository page or talk to Luca Ciampaglia directly.

Future Features
-------
- The following features are currently planned for implementation:
- Code Optimization
- Pneumatic Actuator Support
- Physically-based Sensor Models
- Real-time FESTO System Control

