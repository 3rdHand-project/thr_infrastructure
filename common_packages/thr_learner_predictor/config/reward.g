REWARD {
  tree {
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=1}, aggregate{A, {(at_home A)}, count=2}}, r=105}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=2}, aggregate{A, {(at_home A)}, count=2}}, r=205}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=3}, aggregate{A, {(at_home A)}, count=2}}, r=305}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=4}, aggregate{A, {(at_home A)}, count=2}}, r=405}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=5}, aggregate{A, {(at_home A)}, count=2}}, r=505}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=6}, aggregate{A, {(at_home A)}, count=2}}, r=605}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=1}, aggregate{A, {(at_home A)}, count=1}}, r=102.5}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=2}, aggregate{A, {(at_home A)}, count=1}}, r=202.5}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=3}, aggregate{A, {(at_home A)}, count=1}}, r=302.5}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=4}, aggregate{A, {(at_home A)}, count=1}}, r=402.5}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=5}, aggregate{A, {(at_home A)}, count=1}}, r=502.5}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=6}, aggregate{A, {(at_home A)}, count=1}}, r=602.5}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=1}}, r=100}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=2}}, r=200}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=3}}, r=300}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=4}}, r=400}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=5}}, r=500}
    leaf{{ aggregate{X, Y, Z, {(attached X Y Z)}, count=6}}, r=600}
    weight = 1
  }
#  r=100.
#  { X, Y, Z, { (attached X Y Z) } count=1 }
#  r=200.
#  { X, Y, Z, { (attached X Y Z) } count=2 }
#  r=300.
#  { X, Y, Z, { (attached X Y Z) } count=3 }
#  r=400.
#  { X, Y, Z, { (attached X Y Z) } count=4 }
#  r=500.
#  { X, Y, Z, { (attached X Y Z) } count=5 }
#  r=600.
#  { X, Y, Z, { (attached X Y Z) } count=6 }
#  r=5.
#  { X, { (at_home X) } count=1 }
#  r=10.
#  { X, { (at_home X) } count=2 }
}

FOL_World {
  gamma = 0.9
  stepCost = 0.
  timeCost = 0.
  deadEndCost = 0.
}
