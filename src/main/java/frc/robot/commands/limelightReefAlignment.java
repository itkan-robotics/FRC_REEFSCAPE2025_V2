// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LimelightConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// Code obtained from
// https://www.chiefdelphi.com/t/need-help-with-integrating-pose-estimation-with-apriltags-and-pathplanner-trajectories-in-auto-teleop/455287
public class limelightReefAlignment extends SequentialCommandGroup {
  Drive m_drive;
  LimelightSubsystem m_limelight;

  TagOffsets offset;
  Transform3d TAG_TO_GOAL;
  Pose3d targetPoseEst;
  Pose2d goalPose;

  /***************************************************************************************************************************************************************
   * SequentialCommandGroup that uses Pathplanner AutoBuilder to create a path from the robot's
   * current position to the primary AprilTag's estimated position.
   * <p>(Base code obtained from
   * <a href=https://www.chiefdelphi.com/t/need-help-with-integrating-pose-estimation-with-apriltags-and-pathplanner-trajectories-in-auto-teleop/455287>HERE</a>.
   * Modifications include making it use LimelightHelpers methods instead of PhotonVision)
   * <p> Last Updated by Abdullah Khaled, 2/9/2025
   * @param drive Swerve subsystem
   * @param limelight Limelight subsystem
   * @param offset Where we want to align (i.e. Left, Center, Right). Each enum has a value
   * associated with it that corresponds to how far offset the POI is from the AprilTag
   ***************************************************************************************************************************************************************/
  public limelightReefAlignment(Drive drive, LimelightSubsystem limelight, TagOffsets offset) {
    this.offset = offset;
    this.m_limelight = limelight;
    this.m_drive = drive;
    this.TAG_TO_GOAL =
        new Transform3d(new Translation3d(kReefOffset, 0, 0), new Rotation3d(0, 0, 0.0));

    addRequirements(drive);

    // Stop first because latency compensation is iffy at best
    addCommands(
        new InstantCommand(() -> drive.stop()),
        new WaitCommand(0.5),
        new DeferredCommand(() -> getCommand(), Set.of(drive)),
        new InstantCommand(() -> drive.stop()));
  }

  public Command getCommand() {
    SmartDashboard.putString("Limelight Reef Alignment", "Macro Path");
    if (m_limelight.canSeeTarget() == false) {
      return new InstantCommand();
    } else {

      // Transform the tag's pose to set our goal
      targetPoseEst = new Pose3d(m_limelight.getTagEstimatedPosition(m_drive, offset));

      // Get the pose, making sure it's updated for latency; only works at low speeds though
      // Would like to look into more
      var robotPose2d = m_drive.getPoseLatencyCompensation(m_limelight.getLatency());

      // Convert to Pose3d so we can do maths on the target pose
      var robotPose3d =
          new Pose3d(
              robotPose2d.getX(),
              robotPose2d.getY(),
              0,
              new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()));

      // If we have a valid goal position, set the goal pose to that; otherwise, set the goal pose
      // To the robot's current pose so we don't break anything
      goalPose =
          targetPoseEst.getX() >= 0
              ? robotPose3d
                  .transformBy(new Transform3d(new Pose3d(), targetPoseEst))
                  .transformBy(TAG_TO_GOAL)
                  .toPose2d()
              : robotPose3d.toPose2d();

      // Log the goal pose so we can compare with current pose in AdvantageScope
      Logger.recordOutput("Odometry/Target", goalPose);

      // Configured to work above 12.0V; 11.8V steady is lower bounds
      return AutoBuilder.pathfindToPose(
          goalPose,
          new PathConstraints(5.0, 6.0, Units.degreesToRadians(180), Units.degreesToRadians(75)),
          0);
    }
  }
}
