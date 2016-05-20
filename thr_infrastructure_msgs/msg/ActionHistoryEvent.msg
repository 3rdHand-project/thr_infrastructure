# The action history publishes all started or ended actions (failures + successes) to a topic
# A message of this history is an ActionHistoryEvent

Header header
RobotAction action # The performed action
string side # The arm performing this action

uint8 STARTING = 0
uint8 FINISHED_SUCCESS = 1
uint8 FINISHED_FAILURE = 2
uint8 INSTANTANEOUS = 3
uint8 type # Type of event (one of the constants hereabove)
