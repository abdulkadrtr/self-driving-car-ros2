## Self Driving Car Project

![yz-proje-final](https://github.com/abdulkadrtr/self-driving-car-ros2/assets/87595266/1473a43b-d1fa-426f-aeba-69c99c8a5eae)

This project involves a self-driving car that learns from manual driving data using an artificial neural network model. A route consisting of GPS waypoints is defined for the vehicle, 
and it follows this route using the neural network model. If an obstacle appears while the vehicle is progressing, the neural network model stops the vehicle. Red traffic lights are 
detected using the YOLOv8 model and reported to the neural network, which then stops the vehicle and waits for a green light. The neural network model dynamically makes decisions 
based on environmental conditions, autonomously driving the vehicle to its target destination. For more technical details, please refer to the `report/rapor.pdf` file.

## Demo & Algorithm Description

https://youtu.be/3fzhcxWPNmQ?si=NHlqoIhmli0MQffu

## Installation Instructions


This project has been tested using `Ubuntu 22.04`, `ROS 2 Humble`, and the `Webots` simulation environment. Please ensure that your system meets these requirements.

- First, install the `webots_ros2` packages on your system and verify that the `webots_ros2_tesla` package is functioning correctly.

- Next, load the packages from the `ros2-packages` directory into the `ros2_ws/src` folder. 
The `webots_ros2_tesla` package is a modified Tesla package, while the `autonomous_controller` package is the autonomous driving package. 
Transfer these packages into your working environment and compile them using `colcon build`.

- Remember! You need to install the dependencies for the webots_ros2_tesla package from the original webots_ros2 packages.

- You are now ready to run the autonomous driving application. First, open the simulation environment and the RVIZ2 visualization window by running the command:
  
  `ros2 launch webots_ros2_tesla robot_launch.py`

- Next, start the autonomous driving by running the command:
  
  `ros2 run autonomous_controller autonomous_controller`


  From this point on, you can view feedback related to autonomous driving in the RVIZ2 window and the terminal screen. The vehicle will stop when it sees a red light and wait for it to turn green.
  If an obstacle appears, the vehicle will stop without hitting it and continue once the obstacle is removed.
  When the navigation reaches the main target, the autonomous driving speed will be set to zero, stopping the vehicle.

- Three different GPS routes have been added for autonomous driving test.
  You can track the autonomous driving of the vehicle on different routes by replacing the code line below with `path1`, `path2`, and `path3` respectively.

  https://github.com/abdulkadrtr/self-driving-car-ros2/blob/44907a0afea1c2cd980fb15e2d1e210d733d41ae/ros2-packages/autonomous_controller/autonomous_controller/autonomous_controller.py#L16

## More

- The `autonomous-ai-model` directory contains a dataset with manual driving data and a training file for the neural network model. You are free to use these files as needed.
- In the `detection-model` directory, you can find the training codes for the YOLOv8 model, as well as the Roboflow link for the traffic lights dataset prepared for this project.
  Feel free to utilize these resources accordingly.

## Not found: NeuralNetwork issue solution

Due to the incompatibility between `ROS 2` and `torch.load`, you are likely to encounter this error. 
Therefore, after compiling the project with colcon build, paste the following code into the `install/autonomous_controller/lib/autonomous_controller/autonomous_controller.py` file. 
This will resolve the issue.
```
import torch
import torch.nn as nn
class NeuralNetwork(nn.Module):
    def __init__(self):
        super(NeuralNetwork, self).__init__()
        self.layer1 = nn.Linear(6, 32)
        self.layer2 = nn.Linear(32, 16)
        self.output = nn.Linear(16, 2)

    def forward(self, x):
        x = torch.relu(self.layer1(x))
        x = torch.relu(self.layer2(x))
        x = self.output(x)
        return x
```
