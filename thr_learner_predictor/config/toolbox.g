Terminate
QUIT

# activities
pick
give
hold
go_home_right
go_home_left
wait_for_human
human_can_do

# predicates
attached
positioned
in_human_ws
picked
at_home
busy
free
occupied_slot

holding_position
object

# constants
right
left
/toolbox/handle
/toolbox/side_right
/toolbox/side_left
/toolbox/side_front
/toolbox/side_back
0
1

### RULES

DecisionRule activate_pick {
  Y 
  { (busy left)! (object Y) (picked Y)! (in_human_ws Y)! (free left) }
  { (pick Y)=1 (busy left) (at_home left)! }
}

Rule {
  Y
  { (Terminate pick Y) }
  { (Terminate pick Y)! (pick Y)! (busy left)! (free left)! (picked Y) }
}

DecisionRule activate_give {
  Y
  { (busy left)! (busy right)! (picked Y) }
  { (give Y)=1 (busy left) (at_home left)! }
}

Rule {
  Y
  { (Terminate give Y) }
  { (Terminate give Y)! (give Y)! (busy left)! (picked Y)! (in_human_ws Y) (free left) (at_home left)!}
}

DecisionRule activate_hold {
  X, Z
  { (in_human_ws X) (busy right)! (holding_position Z) (occupied_slot X Z)}
  { (hold X Z)=2 (busy right) (at_home right)! }
}
 
Rule {
  X, Z
  { (Terminate hold X Z) }
  { (Terminate hold X Z)! (hold X Z)! (busy right)! }
}

DecisionRule activate_wait_for_human {
  { (wait_for_human)! (busy left)! (busy right)! }
  { (wait_for_human)=1 (human_can_do) (busy left) (busy right) }
}

Rule {
  { (Terminate wait_for_human) }
  { (Terminate wait_for_human)! (wait_for_human)! (human_can_do)! (busy left)! (busy right)! }
}

DecisionRule activate_go_home_right {
  { (busy right)! (at_home right)!}
  { (go_home_right)=1 (busy right)}
}

Rule {
  { (Terminate go_home_right) }
  { (Terminate go_home_right)! (go_home_right)! (busy right)! (at_home right) }
}

DecisionRule activate_go_home_left {
  { (busy left)! (at_home left)! (free left)}
  { (go_home_left)=1 (busy left)}
}

Rule {
  { (Terminate go_home_left) }
  { (Terminate go_home_left)! (go_home_left)! (busy left)! (at_home left) }
}

#World rules
Rule {
  X, Y, Z
  { (hold X Z) (positioned X Y Z) }
  { (attached X Y Z) }
}

Rule {
  { (human_can_do) (in_human_ws /toolbox/handle) (in_human_ws /toolbox/side_right) (positioned /toolbox/handle /toolbox/side_right 1)!}
  { (human_can_do)! (positioned /toolbox/handle /toolbox/side_right 1) (occupied_slot /toolbox/handle 1) }
}

Rule {
  { (human_can_do) (in_human_ws /toolbox/side_left) (positioned /toolbox/handle /toolbox/side_right 1) (positioned /toolbox/handle /toolbox/side_left 0)!}
  { (human_can_do)! (positioned /toolbox/handle /toolbox/side_left 0) (occupied_slot /toolbox/handle 0) }
}

Rule {
  { (human_can_do) (in_human_ws /toolbox/side_front) (positioned /toolbox/handle /toolbox/side_left 0) (positioned /toolbox/side_left /toolbox/side_front 0)!}
  { (human_can_do)! (positioned /toolbox/side_left /toolbox/side_front 0) (positioned /toolbox/side_right /toolbox/side_front 1) (occupied_slot /toolbox/side_left 0) (occupied_slot /toolbox/side_right 1) }
}


Rule {
  { (human_can_do) (in_human_ws /toolbox/side_back) (positioned /toolbox/side_left /toolbox/side_front 0) (positioned /toolbox/side_left /toolbox/side_back 1)!}
  { (human_can_do)! (positioned /toolbox/side_left /toolbox/side_back 1) (positioned /toolbox/side_right /toolbox/side_back 0) (occupied_slot /toolbox/side_left 1) (occupied_slot /toolbox/side_right 0) }
}


Rule {
  X, Y, Z
  { (positioned X Y Z) (hold X Z)=2 }
  { (attached X Y Z) }
}


### EARLY TERMINATION RULE!

#Rule {
#  X,Y,Z, { (attached X Y Z) }
#  { (QUIT) }
#}

