// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LimelightConstants.ALIGN_KD;
import static frc.robot.Constants.LimelightConstants.ALIGN_KP;
import static frc.robot.Constants.LimelightConstants.LEFT_BRANCH_PIPELINE;
import static frc.robot.Constants.LimelightConstants.TURN_KD;
import static frc.robot.Constants.LimelightConstants.TURN_KP;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoScoreSelection;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LoggedTunableNumber;

/*************************************************************************************
 * Field centric drive command using targetLimelight target data to
 * calculate the desired angle of the robot to align parallel to the target AprilTag
 * and move to the AprilTag based on the target's ta and tx values using Profiled PID.
 * <p> Last Updated by Abdullah Khaled, 1/18/2025
 *************************************************************************************/
public class DriveToReefCommand extends Command {
  Drive drive;
  Timer toleranceTimer;
  AutoScoreSelection storedState;
  LimelightSubsystem lLimelight;
  LimelightSubsystem rLimelight;
  LimelightSubsystem targetLimelight;
  PIDController angleController, extremeAngleController;
  ElevatorSubsystem elevator;
  double slowDownMult = 1.0;
  LoggedTunableNumber alignkP, alignkD, turnkP, turnkD;
  double reefAngle = 91.28;
  boolean finished = false;
  int targetLimelightInt;
  Translation2d linearVelocity = new Translation2d();
  double omega = 0.0;
  String llName;

  /** Creates a new DriveToReefCommand. */
  public DriveToReefCommand(
      Drive d,
      LimelightSubsystem lLimelight,
      LimelightSubsystem rLimelight,
      AutoScoreSelection storedState,
      ElevatorSubsystem e) {
    this.drive = d;
    this.lLimelight = lLimelight;
    this.rLimelight = rLimelight;
    this.storedState = storedState;
    elevator = e;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    slowDownMult =
        elevator.getSlowDownMult(
            Constants.toBotState(storedState.getBotStateInt()).getElevatorSetpoint());
    SmartDashboard.putNumber("slowdownmult", slowDownMult);
    toleranceTimer = new Timer();
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

    targetLimelightInt = storedState.getLimelightTargetPipeline();
    targetLimelight = lLimelight;

    if (targetLimelightInt == LEFT_BRANCH_PIPELINE) {
      targetLimelight = lLimelight;
      llName = LimelightConstants.rightLimelightName;
      // SmartDashboard.putNumber("limelightAlign/int", LEFT_BRANCH_PIPELINE);
    } else {
      targetLimelight = rLimelight;
      llName = LimelightConstants.leftLimelightName;
      // SmartDashboard.putNumber("limelightAlign/int", RIGHT_BRANCH_PIPELINE);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Default values in the case an AprilTag is not seen

    if ((LimelightHelpers.getTV(llName) || toleranceTimer.get() < 0.01)
        && (LimelightHelpers.getFiducialID(llName) == storedState.getTargetAprilTag()
            || LimelightHelpers.getFiducialID(llName) == storedState.getTargetAprilTag() + 11)) {
      toleranceTimer.reset();
      // SmartDashboard.putString("limelightAlign/target", "can see");

      // Get the target angle for the robot based on the AprilTag ID
      reefAngle = targetLimelight.getLLReefAngle(llName);

      if (reefAngle != -1.0) {
        // Calculate angular speed
        if (Math.abs(drive.getRotation().getDegrees() - reefAngle) <= 0.0) {
          omega = extremeAngleController.calculate(drive.getRotation().getDegrees(), reefAngle);
        } else {
          omega = angleController.calculate(drive.getRotation().getDegrees(), reefAngle);
        }

        linearVelocity =
            targetLimelight.getAprilTagVelocity(
                alignkP.getAsDouble(),
                alignkD.getAsDouble(),
                targetLimelightInt,
                reefAngle,
                llName);
      }
      // Correct angle once location reached
    } else if (reefAngle != 91.28 && reefAngle == storedState.getTargetReefAngle()) {
      omega = angleController.calculate(drive.getRotation().getDegrees(), reefAngle);
    }
    // SmartDashboard.putNumber(
    //     "limelightAlign/vxmps",
    //     linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * slowDownMult);
    // SmartDashboard.putNumber(
    //     "limelightAlign/vymps",
    //     linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * slowDownMult);
    // SmartDashboard.putNumber("limelightAlign/omega", omega);

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

    double dynamicTolerance = (0.02 * (Math.cos(Math.toRadians(reefAngle))));
    finished =
        speeds.vxMetersPerSecond <= dynamicTolerance
            && speeds.vyMetersPerSecond <= dynamicTolerance;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
    // targetLimelight.withinTolerance(2.5, 2.5, llName) || finished;
  }
}
