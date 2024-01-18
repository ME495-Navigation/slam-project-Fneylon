# Nuturtle  Description
URDF files for Nuturtle Polaris
* `ros2 launch nuturtle_description load_one.launch.py` to see the robot in rviz.
* `ros2 launch nuturtle_description load_all.launch.xml` to see four copies of the robot in rviz [red, green, blue, purple].

* The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:
![rqt_graph](https://github.com/ME495-Navigation/slam-project-Fneylon/assets/117234679/c1ebc02f-bd09-4d38-874b-a2a7699391a6)
# Launch File Details
* `ros2 launch nuturtle_description load_one.launch.py --show-args`
  `Arguments (pass arguments as '<name>:=<value>'):

    'use_rviz':
        setting the rviz condition
        (default: 'true')

    'use_jsp':
        setting the jsp condition
        (default: 'true')

    'color':
        setting the color of the base_link of the robot. Valid choices are: ['purple', 'red', 'green', 'blue']
        (default: 'purple')`
* `ros2 launch nuturtle_description load_all.launch.xml --show-args`
  `Arguments (pass arguments as '<name>:=<value>'):

    'use_rviz':
        setting the rviz condition
        (default: 'true')

    'use_jsp':
        setting the jsp condition
        (default: 'true')

    'color':
        setting the color of the base_link of the robot. Valid choices are: ['purple', 'red', 'green', 'blue']
        (default: 'purple')

