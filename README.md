# Cooperative assemblies with Baxter

From march 2015 Inria implemented a demonstrator with its Baxter robot illustrating human-robot collaboration in the toolbox assembly task. The global architecture is described here, and the messages can be  [viewed online](thr_infrastructure_msgs/msg).

Our case of study works with a 5-piece wooden toolbox designed for a great number of assembly/disassembly cycles that are: a handle, a front side, a back side, a left side, and a right side. The toolbox has actually 7 pieces, the 2 other composing the bottom of the box, that we do not use in this setup. Each of the 5 pieces is fixed to others using screws, and although they are named differently and recognized as different objects by the system, some parts are actually identical and can be inverted: front = back and left = right. Thus the assembly can differ depending to the location we assemble each side.

The setup is made to execute concurrent actions, the former branch `sequential` was no longer maintained and was thus deleted.

In both branches the robot is only capable of giving an object to the user, and holding an object in place, so that it does move while the user is attaching it. However these high-level actions are implemented differently in both branches and thus lead to different actual low-level actions, that will be detailed later.

**All the objects are documented**, i.e. for each object we know:

- Its **shape** (although it is not used)
- Its **constraints** (the ways it can be attached to other objects)
- A **grasp pose**, i.e. the pose that the vacuum gripper should reach to pick it up
- 2 **hold poses** (one to the left, and one to the right), i.e. poses that the vacuum gripper can reach to hold the object 

**What the robot is not aware of** is the assembly policy and user preferences. The assembly policy tells **when** each object must be assembled, **where** it must be assembled (although the constraints are known, since several pieces are identical, several constraints can be selected) and **where objects must be held** (i.e. select one of the hold poses). These informations missing to the robot are brought to the system through 2 possible policy sources:

- *Either* a hardcoded policy, only for debug purposes
- *Or* via interactive learning
- *Or* using Marc Toussaint's planner
- ([A video illustrating Marc's planner as well as the learner has been sumbmitted to ICRA 2016](https://vimeo.com/139342248])


Marc Toussaint from Stuttgart uni developed a concurrent action planner. Based on his work we defined how concurrent actions are handled by the system, thanks to non-blocking MDP actions, returning always succeeded triggering blocking Robot actions on a compatible arm, which might fail if the robot cannot execute this action. Thus this setup based on Baxter can be used, providing that it's able to perform concurrent actions, hence this branch. The concurrent branch also leads to a faster, more reactive cooperative assembly where no time is lost in systematic GO_HOME actions while new actions are pending.
 
[Click here to open the Dropbox video player][2]

## Implementation

The experimental setup is fully built on ROS, and composed by the following nodes [the brackets contains the ROS type of node]:

 - **Optitrack Publisher** [Publisher]: Generic ROS node previously developed that publishes optitrack rigid bodies in TF frames. Comes with a calibration tool for the bridge ROS <-> Optitrack
 - **Scene State Manager** [Service]: Observes the scene, and creates a representation of the scene made of predicates (see `Perception` below)

 - **Action Server** [Action Server]: Executes an action (pick, go home, hold, ...) onto an agent
 - **Learner/Predictor** [2 Services]: This is the policy provider. It provides two services, one for learning (called each time an action is being run, it can potentially be unused), one for predicting (infinitely called by the interaction controller to get the next action). We have implemented three kinds of Learner/Preductor couples:
  - *Hardcoded predictor* (learner unused)
  - *Marc's planner* (learner unused)
  - *Learner from demonstrations* (the demonstrations need to be recorded first using the keyboard interaction controller)
  - ([A video illustrating Marc's planner as well as the learner has been sumbmitted to ICRA 2016](https://vimeo.com/139342248))
 -  **Interaction controller** [Simple node]: Controller mastering the whole interaction process, requesting the scene state, the actions to perform from the predictor, sending goals to the arms through their action server, and warning the learner than the action succeeded or has been interrupted. We have implemented two kinds of interaction controllers:
  - *Autonomous interaction controller* (automatically looping)
  - *Keyboard interaction controller* (waits for user commands in a Command-Line Interface).


### Action servers
The MDP action server considers both arms as two different agents capable of several actions that take parameters:

- Left arm (vacuum gripper):
 - `PICK(Object obj)`: The robot locates object `obj`, picks it up and keeps it in hand a until further `GIVE` action.

 - `GIVE(Object obj)`: The robot locates locates the human wrist and brings it close to his wrist if it's reachable. When the object is detected to be pulled from the gripper, it is released. The arm does **not** go back in home position.

 - `GO_HOME_LEFT()` The arm goes back in home position, leaving the human workspace.

- Right arm (1-DoF electric gripper):

 - `HOLD(Object obj, HoldPose hp)`: The robot locates object `obj`, approaches its hold pose `#hp`, and grasps the object. When the user wrist leaves far away from `obj`, the action is considered finished, the robot releases the object but does **not** go back in home position.

 - `GO_HOME_RIGHT()` The arm goes back in home position, leaving the human workspace.

There is no `WAIT` action.

### Perception
The perception ability of the robot is ensured by the Optitrack publisher and the Scene State Manager. The scene state is described within a ROS message in the form of predicates. The set of predicates is richer for the concurrent branch as the predictor needs more information about the scene to take decisions.

- `IN_HUMAN_WORKSPACE(Object obj)`: True if `obj` is directly reachable by the human
- `POSITIONED(Object master, Object slave)`: True if the transformation `slave`->`master` matches a documented constraint (i.e. `slave` is positioned close to `master` and is ready to be attached)
- `ATTACHED(Object master, Object slave)`: True if `POSITIONED(master, slave)` and the screwdriver has been seen close to these objects enough time to assume they have been attached together.
- `IN_HOME_POSITION(Arm arm)` True if `arm` is currently in home position
- `BUSY(Arm arm)` True if `arm` is currently performing an operation (recall that there is no `WAIT` operation so sometimes arms perform no action)
- `PICKED(Object obj)` True if `obj` is currently in robot hand
- `HELD(Object obj)` True if `obj` is currently held by robot hand

[1]: https://www.youtube.com/watch?v=9XwqW_V0bDw
[2]: https://www.dropbox.com/s/a6eqy0ziptmniw5/concurrent_coop_assembly.mp4?dl=0

## How to launch the process?

```
roslaunch thr_interaction_controller autonomous.launch policy:=hardcoded display:=debug ip:=<VRPN IP>
```

 *  The policy can be `hardcoded` (default and fastest), `planner` `random` or `policy_player`
 *  The display can be `debug` (default), `action` (displays the actions)
 *  IP is the IP address of the Windows machine running the Optitrack VRPN server (Arena or Motive), the port can also be set (`port:=`).

To start the manual mode (robot receives individual commands from command line):
```
roslaunch thr_interaction_controller manual.launch policy:=hardcoded display:=action ip:=<VRPN IP>
```
Commands are, for instance: `hh0` (Hold the handle, on its left), `hr1` (Hold the piece 'side_right' on its right), `l`, (Send the left arm in home pose), `r` (Send the right arm in home pose), `pb` (Pick the back piece), `gl` (Handover the side_left to the human).

The scene (i.e. objects to use) can be passed in argument. For example the scene `romeo` has been conceived to handle cooking tools, its manual setup uses different letters (see the code of the Keyboard IC to know them):
```
roslaunch thr_interaction_controller manual.launch scene:=romeo policy:=hardcoded display:=action ip:=<VRPN IP>
```


## How to implement new...
## New actions?
The action server works with 2 types of actions: **Non-blocking** [`MDPAction`](thr_infrastructure_msgs/msg/MDPAction.msg)s like `start_xxxxx` causing the MDP action server to trigger a **blocking** [`RobotAction`](thr_action_server/msg/RobotAction.msg) executed by the Robot Action server. The actions themselves are implemented within a [`src/thr_actions/`](thr_action_server/src/thr_actions) subfolder. A new action must then be implemented as:
 *  A new file in this [`src/thr_actions/`](thr_action_server/src/thr_actions) subfoler
 *  A new correspondance `MDP to Robot` in [`config/mdp_robot_mapping.json`](thr_action_server/config/mdp_robot_mapping.json)
 *  A new attribution to the left or right arm in [`config/abilities.json`](thr_action_server/config/abilities.json) (To be merged with mdp_robot_mapping.json?)
 *  Its specific command-line command within the Concurrent Keyboard Interaction Controllers
 *  Its specific tweakable parameters  in [`config/action_params.json`](thr_action_server/config/action_params.json)
 *  Some IK seeds showing some typical arm configuration for this task, in [`config/seeds.json`](thr_action_server/config/seeds.json), if needed
 *  Some text to display to the user on Baxter's screen when the action starts or stops/fails (this file is specific to a scene), in [`config/display.json`](thr_scenes/config/toolbox/display.json), if needed

## New objects/scene?
Scenes are selected thanks to the argument `scene:=toolbox`, all accepted scenes are describes within the `thr_scenes` package.
Adding new objects in a new scene will require to add/change:
 *  The file [`config/scenes.yaml`](thr_scenes/config/scenes.yaml) which describes the available scenes
 *  [`config/<new_scene>/tracked_objects.yaml`](thr_scenes/config/toolbox/tracked_objects.yaml) which describes which objects must be tracked by optitrack, if needed
 *  The file [`config/<new_scene>/poses.json`](thr_scenes/config/toolbox/poses.json) which describes, for its objects, a bunch of hardcoded poses relative to that object 
