# poses.json description

This poses.json stores all poses associated to objects composing the scene:

* poses["individual"] contains all poses related to objects individually
* poses["pairs"] contains poses related to pairs of objects



# action_params.json description

* action_params["sleep_step"] (sec) defines the default sleep duration for low-rate loops 
* action_params["short_sleep_step"] (sec) defines the default sleep duration for higher-rate loops 
* action_params["kv_max"] defines the maximum velocity for trajectory interpolation
* action_params["ka_max"] defines the maximum acceleration for trajectory interpolation
* action_params["action_num_points"] defines the default number of points of a trajectory

* action_params["wait"] contains all the parameters dedicated to action WAIT
* action_params["wait"]["duration"] (sec) defines the duration of a WAIT action

* action_params["give"] contains all the parameters dedicated to action GIVE
* action_params["give"]["sphere_radius"] (meters) defines the radius of the object transfer sphere, if the user wrist leaves the sphere, the robot will reapproach the human wrist with a new goal (orientation does not amtter)
* action_params["give"]["give_pose"] gives the pose where the object is given to the human with respect to his wrist
* action_params["give"]["releasing_disturbance"] (meters) gives the maximum disturbance noise during motion (difference between the effective and the commanded poses). Beyond this threshold, we consider that the human polls th object
* action_params["give"]["inter_goal_sleep"] (sec) gives the sleep duration between two different "Give" goals. In case the human moves and leaves the sphere, the robot will wait this time before reaching the new goal

* action_params["hold"] contains all the parameters dedicated to action HOLD
* action_params["hold"]["sphere_radius"] (meters) gives the radius of the Hold sphere. When his wrist is within this sphere, we consider that the human is working, when it leaves it, action HOLD must terminate 
* action_params["hold"]["approach_cartesian_dist"] (meters) gives the maximum motion of objects to be holded. Above the threshold the robot will recompute its approach pose with a new goal
* action_params["hold"]["approach_angular_dist"] (rad) gives the maximum motion of objects to be holded. Above the threshold the robot will recompute its approach pose with a new goal

* action_params["planning"] contains all the parameters dedicated to planning (if used)
* action_params["planning"]["planner_id"]
* action_params["planning"]["time"] gives the maximum planning time
