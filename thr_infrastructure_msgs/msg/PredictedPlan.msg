# Publishes the predicted plan, i.e. predicted future decisions
# For each decision of this plan, we associate its confidence (0.0 to +inf)

Header header
thr_infrastructure_msgs/Decision[] decisions
float32[] confidences
