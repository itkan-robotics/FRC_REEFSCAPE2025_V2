// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LimelightConstants.ALIGN_KD;
import static frc.robot.Constants.LimelightConstants.ALIGN_KP;
import static frc.robot.Constants.LimelightConstants.TURN_KD;
import static frc.robot.Constants.LimelightConstants.TURN_KP;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoScoreSelection;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

/*************************************************************************************
 * Field centric drive command using limelight target data to
 * calculate the desired angle of the robot to align parallel to the target AprilTag
 * and move to the AprilTag based on the target's ta and tx values using Profiled PID.
 * <p> Last Updated by Abdullah Khaled, 1/18/2025
 *************************************************************************************/
public class DriveToReefCommand extends Command {
  Drive drive;
  AutoScoreSelection buffer;
  LimelightSubsystem limelight;
  PIDController angleController, extremeAngleController;
  double slowDownMult = 0.0;
  LoggedTunableNumber alignkP, alignkD, turnkP, turnkD;
  double reefAngle = 91.28;
  boolean finished = false;

  /** Creates a new DriveToReefCommand. */
  public DriveToReefCommand(
      Drive drive,
      LimelightSubsystem limelight,
      AutoScoreSelection buffer,
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
    turnkP = new LoggedTunableNumber("simpleTurnCtrl/kP", TURN_KP);
    turnkD = new LoggedTunableNumber("simpleTurnCtrl/kD", TURN_KD);

    alignkP = new LoggedTunableNumber("simpleAlignCtrl/kP", ALIGN_KP);
    alignkD = new LoggedTunableNumber("simpleAlignCtrl/kD", ALIGN_KD);

    angleController = new PIDController(turnkP.getAsDouble(), 0.0, turnkD.getAsDouble());
    angleController.enableContinuousInput(-180, 180);

    extremeAngleController =
        new PIDController(turnkP.getAsDouble() * 2.0, 0.0, turnkD.getAsDouble() * 1.05);
    extremeAngleController.enableContinuousInput(-180, 180);
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
      reefAngle = limelight.getLLReefAngle();

      /* To-Do List
       * Test for offset degree #
       *    Add constants to limelightConstants file
       */
      if (reefAngle != -1.0) {
        // Calculate angular speed
        if (Math.abs(drive.getRotation().getDegrees() - reefAngle) <= 0.0) {
          omega = extremeAngleController.calculate(drive.getRotation().getDegrees(), reefAngle);
        } else {
          omega = angleController.calculate(drive.getRotation().getDegrees(), reefAngle);
        }

        // If within certain *arbitrary* turn range drive normally; else, drive slowly
        if (Math.abs(reefAngle - drive.getRotation().getDegrees()) <= 200) {
          linearVelocity =
              limelight.getAprilTagVelocity(
                  alignkP.getAsDouble(), alignkD.getAsDouble(), offsetPipeline, false, reefAngle);
        } else {
          linearVelocity =
              limelight.getAprilTagVelocity(
                  alignkP.getAsDouble(), alignkD.getAsDouble(), offsetPipeline, true, reefAngle);
        }
      }
    } else if (reefAngle != 91.28 && reefAngle == buffer.getTargetReefAngle()) {
      omega = angleController.calculate(drive.getRotation().getDegrees(), reefAngle);
    }
    limelight.withinTolerance(0.1, 0.2);
    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * slowDownMult,
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * slowDownMult,
            omega);
    finished = speeds.vxMetersPerSecond <= 0.1;
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
    return limelight.withinTolerance(2.5, 2.5) || finished;
  }
}
