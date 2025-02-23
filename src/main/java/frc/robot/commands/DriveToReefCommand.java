// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.BufferSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

/*************************************************************************************
 * Field centric drive command using limelight target data to
 * calculate the desired angle of the robot to align parallel to the target AprilTag
 * and move to the AprilTag based on the target's ta and tx values using Profiled PID.
 * <p> Last Updated by Abdullah Khaled, 1/18/2025
 *************************************************************************************/
public class DriveToReefCommand extends Command {
  Drive drive;
  BufferSubsystem buffer;
  LimelightSubsystem limelight;
  PIDController angleController;
  double slowDownMult = 0.0;
  /** Creates a new DriveToReefCommand. */
  public DriveToReefCommand(
      Drive drive,
      LimelightSubsystem limelight,
      BufferSubsystem buffer,
      DoubleSupplier slowDownMultSupplier) {
    this.drive = drive;
    this.limelight = limelight;
    this.buffer = buffer;
    slowDownMult = slowDownMultSupplier.getAsDouble();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Create PID controller
    angleController =
        new PIDController(LimelightConstants.TURN_KP, 0.0, LimelightConstants.TURN_KD);
    angleController.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Default values in the case an AprilTag is not seen
    Translation2d linearVelocity = new Translation2d();
    double omega = 0.0;
    int offsetPipeline = buffer.getTargetPipeline();

    if (limelight.canSeeTarget() && limelight.getID() == buffer.getTargetAprilTag()) {

      // Get the target angle for the robot based on the AprilTag ID
      double reefAngle = limelight.getLLReefAngle();

      /* To-Do List
       * Test for offset degree #
       *    Add constants to limelightConstants file
       */
      if (reefAngle != -1.0) {
        // Calculate angular speed
        omega = angleController.calculate(drive.getRotation().getDegrees(), reefAngle);

        // If within certain *arbitrary* turn range drive normally; else, drive slowly
        if (Math.abs(reefAngle - drive.getRotation().getDegrees()) <= 20) {
          linearVelocity = limelight.getAprilTagVelocity(offsetPipeline, false, reefAngle);
        } else {
          linearVelocity = limelight.getAprilTagVelocity(offsetPipeline, true, reefAngle);
        }
      }
    }

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * slowDownMult,
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * slowDownMult,
            omega);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limelight.withinTolerance(0.1, 0.2);
  }
}
