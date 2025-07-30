# Tetherbots PyBullet Simulation

This repository holds the PyBullet simulation framework for realistically simulating the control algorithm and physical behavior of coordinated tether-connected robots that have minimal access to global sensing information beyond the strain and orientation of its connected tether(s).

## Implementation Notes

- All functions take in angle measurements in degrees and return angle measurements in degrees for better intuition.

- All units for inputs and outputs are in SI units (kg, newtons, meters, seconds, etc.) unless stated otherwise in comments or documentation.

- All tetherbots and obstacles are situated on planar joints (technically two prismatic joints stacked on each other because PyBullet does not support planar joints directly) with a continuous joints that enable on-axis rotation. This is to restrict their motion and physics to 2D.

## PyBullet Notes

- PyBullet measures global heading of an object starting with 0 on the positive x-axis and grow more positive in the counter-clockwise direction and more negative in the clockwise direction. It is unbounded, meaning that the angle measurement accumulates on every rotation and can grow far beyond 360 or -360 degrees. 

- In the visualizer GUI, hold Ctrl or Alt and drag the screen with the left mouse button to rotate the camera view. Drag the screen with the middle mouse button to translate camera view.

## Agent Class

The Agent class implemented in agent.py represents a tetherbot object. The agent class represents a tetherbot in the sense that it should be, as in what it would do and have knowledge of in the real world.
