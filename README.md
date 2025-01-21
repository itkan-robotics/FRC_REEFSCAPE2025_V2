<h1> How I'm Hoping to Implement Limelight 3D Into the Current Code:</h1>
1. We'll have to create a new pipeline that configures the limelight's position and rotation relative to the robot; this should be relatively straightforward as an example is shown in the documentation shown below:
  <br> - [https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-3d#full-3d-tracking](url)
<br> 2. Now that the camera knows where it is in relation to the robot, it can calculate the AprilTag's position relative to the robot using
      <br> NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        <br> - "3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees)" ([https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data](url))
  <br> - This way, we *should* be able to find the position of the robot relative to the primary AprilTag in view (This saves us the hassle of figuring that out ourselves).
<br> 3. Since we have the location of the robot relative to the target AprilTag, we can utilize PID control to drive to the AprilTag, using tz for the range and tx/ty to calculate the angle, as well as align with the AprilTag, using the yaw of the AprilTag relative to the robot.
<br> 4. Finally, we can set up multiple pipelines; one for the left branch, one for centering, and one for the right branch; this would allow us to offset the robot from the AprilTag without having to manually adjust values.

<h1>Reef auto zones, leave paths, and AprilTag IDs; each ID has an 'L' and 'R' branch, for which I have code written for both </h1>

![Reef auto zones, leave paths, and AprilTag IDs](https://github.com/user-attachments/assets/e69f752e-a2b8-45d5-b349-b5c5bb180dea)
