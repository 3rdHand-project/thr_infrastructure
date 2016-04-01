cmake_minimum_required(VERSION 2.8.3)
project(thr_infrastructure_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
   std_msgs
   geometry_msgs
   message_generation
   message_runtime
   actionlib_msgs
  )

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Decision.msg
   RobotAction.msg
   Predicate.msg
   SceneState.msg
   ActionHistoryEvent.msg
   PredictedPlan.msg
   UserAction.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   GetNextDecision.srv
   GetSceneState.srv
   SetNewTrainingExample.srv
   UpdateRelationalState.srv
   StartStopEpisode.srv
   GetCarryPose.srv
 )

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   RunDecision.action
   RunRobotAction.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   actionlib_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES thr_coop_assembly
  CATKIN_DEPENDS
   std_msgs
   geometry_msgs
   message_generation
   message_runtime
   actionlib_msgs
#  DEPENDS system_lib
)
