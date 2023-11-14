# OrientationControl
This project proposes a force-based task-orientation controller to online deal with uncertain interaction tasks. A gradient descent-based orientation controller is proposed, enhancing its performance with orientation predictions provided by a Gaussian Process modeling.

# Gradient descent orientation control (high frequency)
orientation_ctrl.cpp can be used for the low-level Cartesian impedance control with gradient descent-based orientation adaptation. It should be included within the franka_example_controllers.

# GP-based orientation control (low frequency)
bo_orientation_ctrl is the ROS package for the GP-based low-frequency update of the orientation setpoint of the impedance control. It subscribes to topics published by orientation_ctrl and it publishes topics subscribed by orientation_ctrl.
The package makes use of the limbo c++ library for GP implementation (http://resibots.eu/limbo/).

# Paper
Loris Roveda, Marco Pavone. Gradient Descent-Based Task-Orientation Robot Controller Enhanced with Gaussian Process Predictions.

# Video
A video describing the methodology and the achieved results (including experiments) can be found here: https://youtu.be/5BmknNo0uXg?feature=shared
