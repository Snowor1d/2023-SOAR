
This is 2023 SOAR path planning team's workspace.

'soar_control_stil.py' runs normally only in mission gazebo world(for 2023 korea robot airplan contest).
it have to run with yolov8 detection node, map_talker node, object position detect node.

In 'soar_control ~ .py', it contains high-level control algorithm, particular path control algorithm, and total path-planning algorithms for mission.

SE_algorithm (python and c++ file) is designed for obstacle avoidance (particularly move from one to another while avoiding two ladder).

In, 'map_talker.py', it publish obstacle informations using voxelization and clustering (k-means algorithm)

All of this have to be operated in px4_autopilot development environment.

![simulation1](https://github.com/Snowor1d/2023-SOAR/assets/96639889/62bdef5d-88eb-415e-bd6a-5b113b87bd0f)



![go path](https://github.com/Snowor1d/2023-SOAR/assets/96639889/7f017e69-e03b-48d6-aa9f-dd5147faac3e)




![soar_simulation_software_structure](https://github.com/Snowor1d/2023-SOAR/assets/96639889/602c798c-1476-46c0-958a-3a57c90b8c43) 



![Screenshot from 2023-07-17 21-50-29](https://github.com/Snowor1d/2023-SOAR/assets/96639889/99d6ec07-7067-414c-92b3-2c524dbcd7f7)



HAVE A NICE DAY :)


