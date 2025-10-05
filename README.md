<h1> Understanding the 9128 2025 (REEFSCAPE) Robot </h1>
<br>
<h2>Command-Based Structure</h2>
For our programming, we chose to use a command-based structure. This type of code has three main components: subsystems, commands, and controllers.
<br> <br>
<ol>
  <li><b></b>Subsystems:</b> These are the various major components of the robot. This year, we had three main subsystems: our <b></b>drivetrain, arm, and intake.</ol> For each subsysystem, we defined the motors and sensorsused by that subsystem along with the subsystem-specific functions using those motors and sensors. Something unique we did with our arm, however, (which we took inspiration from 2910's 2023 code) is that we contained all three "joints" of the arm (i.e. shoulder, extension, and wrist) in the same subsystem. We did this so that it was easier to control the arm rather than trying to control them using a command (which we found less intuitive than simply combining them into the same subsystem). <br><br></li>
  <li><b>Commands:</b> These are the actions we want the robot to do, and can be either subsystem-specific or involve the interaction of multiple subsystems. For example, for our arm subsystem, we had one command that set the position of the shoulder, extension, and wrist components all at the same time. However, for our auto-align, we needed not only the drivetrain to move towards the target, but also the arm to extend when it was within a certain distance calibrated to not only prevent our robot from colliding with the reef, but also from tipping or even just swaying excessively. <br><br></li>
  <li><b>Controllers:</b> Finally, we have our controllers, which is what allows the robot to act under driver operation during the tele-operated period. We used a dual-controller setup, with the driver's controller deciding the majority of the robot's actions and the operator's controller only controlling the outtake of coral and the various parameters for the auto-align to relieve the driver of some stress. <br><br></li>

![Subsystems](https://github.com/user-attachments/assets/b82bb05c-d61c-43c4-931d-c0da9b75861a)

<h6>Figure showing the command-based structure</h6>

</ol>
<h2>Design Highlights</h2>
<br>
<h3>Machine States Storage</h3>
Due to the large amount of different states our robot could have, from intaking to L1-4 to de-algaefying both high and low algaes, we realized that we needed a way to store various information about these states and have them easily accessible throughout the program. Therefore, we created the Machine States class, which stores not only the positions of <b>all three parts of our arm</b>, but also <b>outtaking speeds</b>, each position's <b>name</b>, and its <b>ID.</b> This way, if we want to create new positions, add new state-specific features, or configure each position individually, there is one class that allows us to do all of that.
<br>
<h3>Autonomous Pathing</h3>
Another aspect of our machine that we wanted to highlight was our <b>autonomous pathing</b> and general auto routines. We used <b>PathPlanner</b>, a path-creating tool that uses Bézier curves to create smooth, continuous motion of holonomic machines (i.e. swerve drives like our robot). However, not only did we create smooth paths (after carefully tweaking various constraints, such as the maximum translational and rotational velocity and acceleration of our robot), we used <b>event markers</b> to preemptively move our arm to the correct scoring our intaking position, saving us valuable time during auto. Additionally, we used <b>constraint zones</b> to balance speed during long streches of movement with precision when we got close to picking up a coral or scoring it on the reef, making our auto more consistent that it would have been without them.
<br>
<h3>Smart Auto-Align</h3>
The final unique aspect of our programming (that we wanted to highlight!) was our auto align. It used a <b>three-step process</b> to get the robot to the exact location and angle it needed to within ±5º of heading error and ±2 in. of horizontal error; operator input storage; target pose estimation and profiled PID translational and angular velocity control; and dynamic tolerances based on operator inputs.
<br> <br>
<ol>
  <li><b>Operator Input Storage:</b> From the start of the season, our team understood the importance of automatically aligning to the reef with as little input from the driver as possible. However, we found that the operator trying to press three different buttons (branch, level, and angle) at the same time was unwieldly and unrealistic, especially during the height of the competition. As such, we realized that we needed to create a separate class to store the operator's inputs once they selected them, which we did through our AutoScoreSelection class. This way, the operator can select the next place for the auto-align to score on as soon as the coral is scored.</li>
  <li><b>Target Pose Estimation and Velocity Control:</b> Throughout the season, we had various approaches to try to align accurately and precisely to the reef.
  <ol>
    <li>We had <b>three separate PID controllers</b> for the x speed, y speed, and heading. We used the tx and ta values to determine the x and y relative distances from the AprilTag and a HashMap to determine the desired reef angle from the primary AprilTag visible. However, we found that this solution was not precise enough for our needs, as it was inconsistent and was often outside of our allowable tolerance.</li>
    <li>We attempted to use <b>full-field localization</b> using Limelight's built-in vision calculations and <b>Pathplanner's holonomic drive controller</b> to create smooth and accurate paths to the target AprilTag. While this was a better solution than before, it was slow to start (as a result of the Profiled PID controller having max acceleration limits), and we felt that we had not explored all of our options.</li>
    <li>We decided to use one <b>ProfiledPID controller for translation and one PID controller for heading</b>, taking after other teams such as <a href="https://github.com/Mechanical-Advantage">Mechanical Advantage</a>. To determine the x and y speeds (as the translation controller only gave magnitude, not direction), we normalized the x and y magnitudes baesd on the output magnitude (i.e. the farther the robot was laterally, the faster the robot will move laterally). Additionally, we calculated the heading by rounding the robot's heading to the nearest 60º, since we assumed our drivers would align about 80% correctly with the reef face. This combination of solutions gave us the accuracy, speed, and precision we desired, and it ultimately became our solution to our auto align.</li>
  </ol></li>
  <li><b>Dynamic Tolerances:</b> Finally, we added dynamic tolerances based on the robot's distance to the target for when the <b>arm should move</b> to the desired level and when the auto-align is <b>considered finished.</b> For our arm, we found that our L2 position needed to be farther back than that of L3 or L4. By using dynamic tolerances, we were able to accomodate for this problem and ensure a consistent score on all three levels.</li>
</ol>

<h6> <i>Written by Abdullah Khaled</i> </h6>
