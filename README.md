# Video Groundtruth

ROS repository for estimating a pose groundtruth based on exploiting external video data

IDEA: Record video of intersection using camera with known intrinsics and minimal distortion. Estimate the pose on the robot based on a tag of scene-unique color, which is fixed to the robot, getting scale information from the known structure of the intersection area.

TODO: 
- stabilizing strip detection (current: color, shape) --> other more unique color than dark red, apriltag ?
- convert pixel to world coordinate using intersection structure based scale
