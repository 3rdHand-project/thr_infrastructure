# Inria march 2015: Cooperative assemblies with Baxter

From march 2015 Inria implemented a demonstrator with its Baxter robot illustrating human-robot collaboration in the toolbox assembly task. The global architecture is described here, and the messages can be  [viewed online](https://github.com/3rdHand-project/Inria/tree/master/common_packages/thr_coop_assembly/msg).

Our case of study works with a 5-piece wooden toolbox designed for a great number of assembly/disassembly cycles that are: a handle, a front side, a back side, a left side, and a right side. The toolbox has actually 7 pieces, the 2 other composing the bottom of the box, that we do not use in this setup. Each of the 5 pieces is fixed to others using screws, and although they are named differently and recognized as different objects by the system, some parts are actually identical and can be inverted: front = back and left = right. Thus the assembly can differ depending to the location we assemble each side.

The setup splits in 2 branches that we will both maintain as much as possible:

 - A non-concurrent branch: the robot performs one action at a time
 - A concurrent branch: the robot performs several actions at a time (as much as available end effectors... 2 for Baxter)

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


## March-May: The non-concurrent branch
The non-concurrent setup was the first coded in time, and allows only one Baxter arm moving at a time. It means that even when an arm has finished its master action (e.g. holding some piece) and is currently returning at its home position, the whole robot is immobilized performing this secondary action, which does not look so natural.

### illustration of the non-concurrent setup
----------------------------------------------------

[Click here to open the Dropbox video player][2]

## May: The concurrent branch
Marc Toussaint from Stuttgart uni is working on a concurrent action planner which Thibaut could reuse in the relational domain (see his weekly for precise scientific infos). Thus this setup based on Baxter can be used, providing that it's able to perform concurrent actions, hence this branch. The concurrent branch also leads to a faster, more reactive cooperative assembly where no time is lost in systematic GO_HOME actions while new actions are pending.
 
### illustration of the concurrent setup
----------------------------------------------------

[Click here to open the Dropbox video player][3]

## Implementation

The experimental setup is fully built on ROS, and composed by the following nodes [the brackets contains the ROS type of node]:

 - **Optitrack Publisher** [Publisher]: Generic ROS node previously developed that publishes optitrack rigid bodies in TF frames. Comes with a calibration tool for the bridge ROS <-> Optitrack
 - **Scene State Manager** [Service]: Observes the scene, and creates a representation of the scene made of predicates (see `Perception` below)

 - **Action Server** [Action Server]: Executes an action (pick, go home, hold, ...) onto an agent (1 server for the non-concurrent setup, 2 servers = one for each arm for the concurrent one)
 - **Learner/Predictor** [2 Services]: This is the policy provider. It provides two services, one for learning (called each time an action is being run, it can potentially be unused), one for predicting (infinitely called by the interaction controller to get the next action). We have implemented three kinds of Learner/Preductor couples:
  - *Hardcoded predictor* (learner unused)
  - *Marc's planner* (learner unused)
  - *Learner from demonstrations* (the demonstrations need to be recorded first using the keyboard interaction controller)
 -  **Interaction controller** [Simple node]: Controller mastering the whole interaction process, requesting the scene state, the actions to perform from the predictor, sending goals to the arms through their action server, and warning the learner than the action succeeded or has been interrupted. We have implemented two kinds of interaction controllers:
  - *Autonomous interaction controller* (automatically looping)
  - *Keyboard interaction controller* (waits for user commands in a Command-Line Interface).


### Action servers
#### Non-concurrent branch
In the non-concurrent branch the robot is a single agent capable of several actions that take parameters:

- `GIVE(Object obj)`: The robot locates object `obj`, picks it up, locates the human wrist and brings it close to his wrist if it's reachable. When the object is detected to be pulled from the gripper, it is released and the arm returns to the home position. This action is affected to the vacuum gripper.

- `HOLD(Object obj, HoldPose hp)`: The robot locates object `obj`, approaches its hold pose `#hp`, and grasps the object. When the user wrist leaves far away from `obj`, the action is considered finished, the robot releases the object and goes back in home position. This action is affected to the vacuum gripper 

- `WAIT()`: This action sleeps 1 second. This action has no affected arm.

We consider that an arm always performs one action, at least `WAIT()` if it does nothing, hence the policy source must always return an action.

#### Concurrent branch
The concurrent branch considers both arms as two different agents capable of several actions that take parameters:

- Left arm (vacuum gripper):
 - `PICK(Object obj)`: The robot locates object `obj`, picks it up and keeps it in hand a until further `GIVE` action.

 - `GIVE(Object obj)`: The robot locates locates the human wrist and brings it close to his wrist if it's reachable. When the object is detected to be pulled from the gripper, it is released. The arm does **not** go back in home position.

 - `GO_HOME_LEFT()` The arm goes back in home position, leaving the human workspace.

- Right arm (1-DoF electric gripper):

 - `HOLD(Object obj, HoldPose hp)`: The robot locates object `obj`, approaches its hold pose `#hp`, and grasps the object. When the user wrist leaves far away from `obj`, the action is considered finished, the robot releases the object but does **not** go back in home position.

 - `GO_HOME_RIGHT()` The arm goes back in home position, leaving the human workspace.

There is no more `WAIT` action but the policy source (hardcoded or learner/predictor) is allowed to return an empty list of actions (when the assembly is complete).

### Perception
The perception ability of the robot is ensured by the Optitrack publisher and the Scene State Manager. The scene state is described within a ROS message in the form of predicates. The set of predicates is richer for the concurrent branch as the predictor needs more information about the scene to take decisions.
- Base predicates:
 - `IN_HUMAN_WORKSPACE(Object obj)`: True if `obj` is directly reachable by the human
 - `POSITIONED(Object master, Object slave)`: True if the transformation `slave`->`master` matches a documented constraint (i.e. `slave` is positioned close to `master` and is ready to be attached)
 - `ATTACHED(Object master, Object slave)`: True if `POSITIONED(master, slave)` and the screwdriver has been seen close to these objects enough time to assume they have been attached together.
- additional predicates for the concurrent branch:
 - `IN_HOME_POSITION(Arm arm)` True if `arm` is currently in home position
 - `BUSY(Arm arm)` True if `arm` is currently performing an operation (recall that there is no `WAIT` operation so sometimes arms perform no action)
 - `PICKED(Object obj)` True if `obj` is currently in robot hand
 - `HELD(Object obj)` True if `obj` is currently held by robot hand

[1]: https://www.youtube.com/watch?v=9XwqW_V0bDw
[2]: https://www.dropbox.com/s/6klnlafpoki0cs2/coop_assembly.mp4?dl=0
[3]: https://www.dropbox.com/s/a6eqy0ziptmniw5/concurrent_coop_assembly.mp4?dl=0
