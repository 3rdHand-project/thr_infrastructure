REWARD {
  r=100.
  { X, Y, Z, { (attached X Y Z) } count=1 }
  r=200.
  { X, Y, Z, { (attached X Y Z) } count=2 }
  r=300.
  { X, Y, Z, { (attached X Y Z) } count=3 }
  r=400.
  { X, Y, Z, { (attached X Y Z) } count=4 }
  r=500.
  { X, Y, Z, { (attached X Y Z) } count=5 }
  r=600.
  { X, Y, Z, { (attached X Y Z) } count=6 }
  r=5.
  { X, { (at_home X) } count=1 }
  r=10.
  { X, { (at_home X) } count=2 }
}

FOL_World {
  gamma = 0.9
  stepCost = 0.
  timeCost = 0.
  deadEndCost = 0.
}
