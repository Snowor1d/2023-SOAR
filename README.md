
This is 2023 SOAR path planning team's workspace.

'soar_control_stil.py' runs normally only in mission gazebo world(for 2023 korea robot airplan contest).
it have to run with yolov8 detection node, map_talker node, object position detect node.

In 'soar_control ~ .py', it contains high-level control algorithm, particular path control algorithm, and total path-planning algorithms for mission.

SE_algorithm (python and c++ file) is designed for obstacle avoidance (particularly move from one to another while avoiding two ladder).

In, 'map_talker.py', it publish obstacle informations using voxelization and clustering (k-means algorithm)

All of this have to be operated in px4_autopilot development environment.

![simulation1](https://github.com/Snowor1d/2023-SOAR/assets/96639889/62bdef5d-88eb-415e-bd6a-5b113b87bd0f)


HAVE A NICE DAY :)


