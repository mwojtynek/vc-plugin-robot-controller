# vc-plugin-robot-controller

A robot plugin for the commercial process simulation [Visual Components](https://visualcomponents.com).<br>
The plugin adds the capability of path planning, collision avoidance, and soft real-time motion control.
The easy-to-use high-level python interface in the simulation GUI allows access to the low-level controller. 

A use-case was implemented for speed-and-separation monitoring to interrupt the current robot motion at the human presence:
https://youtu.be/zcMzQt0ByGs

#### IActionItem <br>
`provides access from the highlevel Python programming API to the low-level infrastructure`<br>

#### MotionPlanningManager<br>
`For a convenient operating with the robot, the MotionPlanningManager organizes the motion requests from one or multiple robots in the simulated scene. 
At each simulationrun, the robots are instantiated and the 3D scene is passed to perform path planning and collision avoidance on the current simulation layout.
A motion request between a pair of task frames is managed at run-time by the MotionPlanningManager for each partial movement of a robot task.`<br>

#### ISimPlugin <br>
`is used to get full access to the simulation scene, the properties of a component and the user-interface.`<br>

#### RobotController<br>
`updates the robot’s joint angles at each timestep.`

#### MotionInterpolator<br>
`slices a trajectory into time-based steps, that are executed sequentially by the RobotController.`<br>

#### RobotParameter<br>
`Robot specific parameters such as the 3D-model of the robotic manipulator, upper and lower joint limits, or the robot position in the simulated world are stored
and used from the data structure of the RobotParameter class.`<br>

<img src="https://github.com/mwojtynek/vc-plugin-robot-controller/blob/main/vc-plugin-class-diagram.png" alt="drawing" width="750"/>
