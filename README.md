# Wall-Climbing Robot Simulation (PyBullet)

Keyboard-controlled PyBullet simulation of a VertiGo-inspired dual-fan wall-climbing robot. Built as part of an academic team project, *Vertical Mobility: Design and Simulation of a Wall-Climbing Robot*, under faculty supervision.

The robot's SolidWorks model (exported to URDF) was made by a teammate. The simulation script is my own work.

## Demo
[wcr_demo.mp4](wcr_demo.mp4)

## What it does
- Drives on the floor, climbs the front face of a wall, and climbs the back face
- Floor / front wall / back wall state machine with 90-degree transitions in both directions
- Front and rear fans tilt independently, in the style of VertiGo
- Live on-screen display of surface, position, thrust and fan angles

## Limitations
This is a scripted simulation, not an aerodynamic model. The robot's pose is set directly each frame, and wall adhesion is a constant pressing force. It demonstrates the control logic and the VertiGo concept, not real fan thrust.

## Run it
```
pip install pybullet numpy
python wcr_simulation.py
```

Keep the folder layout below, because the script loads the URDF from `wcr/urdf/wcr.urdf` and the URDF loads its meshes from `wcr/meshes/`.

```
wcr_simulation.py
wcr/
  urdf/wcr.urdf
  meshes/*.STL
```

Controls: arrow keys to drive, climb and turn; Q to quit.

## Tools
Python, PyBullet, NumPy, URDF
