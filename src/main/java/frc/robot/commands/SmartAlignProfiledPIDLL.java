// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LimelightConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.FullArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoScoreSelection;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.MachineStates.BotState;
import frc.robot.util.TuneableProfiledPID;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartAlignProfiledPIDLL extends Command {
  /** Creates a new SmartAlign. */
  Drive m_drive;

  AutoScoreSelection storedState;
  FullArmSubsystem arm;
  LimelightSubsystem lLimelight, rLimelight;
  private PIDController m_thetaController;
  private String limelightName;
  BotState currentState;

  // TuneableProfiledPID m_profiledPidY, m_profiledPidX;
  TuneableProfiledPID m_profiledPidTrans;
  Pose2d targetPoseRobotRelative = new Pose2d();

  boolean isAngleReached = true;
  boolean isStrafeReached = false;

  double targetReefAngle = 0.0;

  double omega = 0.0;
  double xTrans = 0.0;
  double yTrans = 0.0;
  double count = 0.0;
  double rotationVal = 0.0;

  double desiredArea = 11.8;

  boolean m_end = false;
  boolean rotationReached = false;
  int tagID = 10;
  double reefAngle = 0.0;

  public SmartAlignProfiledPIDLL(Drive d, FullArmSubsystem a, AutoScoreSelection b) {
    this.m_drive = d;
    this.storedState = b;
    this.arm = a;
    addRequirements(m_drive, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = storedState.getBotState();

    m_end = false;
    double targetLimelightInt = storedState.getLimelightTargetPipeline();

    if (targetLimelightInt == LEFT_BRANCH_PIPELINE) {
      limelightName = LimelightConstants.rightLimelightName;
    } else if (targetLimelightInt == RIGHT_BRANCH_PIPELINE) {
      limelightName = LimelightConstants.leftLimelightName;
    }
    limelightName = LimelightConstants.rightLimelightName;

    // m_profiledPidY = new TuneableProfiledPID("m_profiledPidY", 0.14, 0.0, 0.005, 3.0, 3);
    // m_profiledPidY.setTolerance(0.5);

    // m_profiledPidX = new TuneableProfiledPID("m_profiledPidX", 0.04, 0.0, 0.001, 3.0, 3.0);
    // m_profiledPidX.setTolerance(0.5);

    m_profiledPidTrans = new TuneableProfiledPID("m_profiledPidTrans", 0.1, 0.0, 0.0, 3.0, 3.0);
    m_profiledPidTrans.setGoal(0);
    m_profiledPidTrans.setTolerance(0.5);

    m_thetaController = new PIDController(0.12, 0.0, 0.0);
    m_thetaController.enableContinuousInput(-180, 180);
    m_thetaController.setTolerance(3);

    isStrafeReached = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_profiledPidX.updatePID();
    // m_profiledPidY.updatePID();
    m_profiledPidTrans.updatePID();

    // double[] targetPoseArr = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
    double[] targetPoseArr = LimelightHelpers.getTargetPose_CameraSpace(limelightName);

    targetPoseRobotRelative =
        new Pose2d(targetPoseArr[0], -targetPoseArr[1], Rotation2d.fromDegrees(targetPoseArr[4]));

    SmartDashboard.putNumberArray(
        "targetPoseBefore",
        new double[] {
          Units.metersToInches(targetPoseRobotRelative.getX()),
          Units.metersToInches(targetPoseRobotRelative.getY()),
          targetPoseRobotRelative.getRotation().getDegrees()
        });

    targetPoseRobotRelative =
        new Pose2d(
            targetPoseRobotRelative.getX(),
            targetPoseRobotRelative.getY()
                + Units.inchesToMeters(-4.5)
                - Units.inchesToMeters(7.25),
            targetPoseRobotRelative.getRotation());

    SmartDashboard.putNumberArray(
        "targetPoseAfter",
        new double[] {
          Units.metersToInches(targetPoseRobotRelative.getX()),
          Units.metersToInches(targetPoseRobotRelative.getY()),
          targetPoseRobotRelative.getRotation().getDegrees()
        });

    // SmartDashboard.putNumberArray("targetPoseArray", targetPoseArr);

    if (LimelightHelpers.getTV(limelightName)) {
      //   if (isAngleReached) {

      double dist = targetPoseRobotRelative.getTranslation().getDistance(new Translation2d(0, 0));
      double targetLateralOffset = targetPoseRobotRelative.getX() - 0;
      double targetVerticalOffset = targetPoseRobotRelative.getY() - 0;

      SmartDashboard.putNumber("llsmartalign/dist", dist);
      SmartDashboard.putNumber("llsmartalign/lat", targetLateralOffset);
      SmartDashboard.putNumber("llsmartalign/vert", targetVerticalOffset);

      rotationVal =
          m_thetaController.calculate(
              m_drive.getHeadingDegrees(), Math.round(m_drive.getHeadingDegrees() / 60) * 60);

      double controlValue = m_profiledPidTrans.calculate(dist);

      SmartDashboard.putNumber("controlVal", controlValue);

      xTrans = controlValue * (targetLateralOffset / dist);
      yTrans = controlValue * (targetVerticalOffset / dist);

      xTrans = MathUtil.clamp(xTrans, -10, 10);
      yTrans = MathUtil.clamp(yTrans, -10, 10);

      if (isAngleReached) {
        MathUtil.clamp(rotationVal, -0.75, 0.75);
      }
      if (isStrafeReached) {
        MathUtil.clamp(xTrans, -1.0, 1.0);
      }

      yTrans = MathUtil.applyDeadband(yTrans, 0.2);
      xTrans = MathUtil.applyDeadband(xTrans, 0.2);
      rotationVal = MathUtil.applyDeadband(rotationVal, 0.05);
    }

    ChassisSpeeds speeds = new ChassisSpeeds(yTrans, -xTrans, rotationVal);
    SmartDashboard.putNumber("ProfiledPIDX", xTrans);
    SmartDashboard.putNumber("ProfiledPIDY", yTrans);
    SmartDashboard.putNumber("RotationValue", rotationVal);

    m_drive.runVelocity(speeds);

    if (m_profiledPidTrans.atSetpoint() && count > 25) {
      m_end = true;
    } else {
      m_end = false;
    }
    // }

    SmartDashboard.putBoolean("isTarget", LimelightHelpers.getTV(limelightName));

    count++;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, m_drive.getRotation()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
