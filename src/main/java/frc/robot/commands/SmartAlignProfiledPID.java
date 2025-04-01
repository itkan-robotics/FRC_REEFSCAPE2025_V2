// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LimelightConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
public class SmartAlignProfiledPID extends Command {
  /** Creates a new SmartAlign. */
  Drive m_drive;

  AutoScoreSelection storedState;
  FullArmSubsystem arm;
  LimelightSubsystem lLimelight, rLimelight;
  private PIDController m_thetaController;
  private String limelightName;
  BotState currentState;

  TuneableProfiledPID m_profiledPidY, m_profiledPidX;

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

  public SmartAlignProfiledPID(Drive d, FullArmSubsystem a, AutoScoreSelection b) {
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
    // SmartDashboard.putNumber("autoAlignTeleop/targetLimelightInt", targetLimelightInt);

    if (targetLimelightInt == LEFT_BRANCH_PIPELINE) {
      limelightName = LimelightConstants.rightLimelightName;
    } else if (targetLimelightInt == RIGHT_BRANCH_PIPELINE) {
      limelightName = LimelightConstants.leftLimelightName;
    }

    m_profiledPidY = new TuneableProfiledPID("m_profiledPidY", 0.17, 0.0, 0.005, 7.0, 10);
    // m_profiledPidY.setGoal(16.0);
    m_profiledPidY.setTolerance(0.5);
    // m_pidControllerY = new PIDController(0.7, 0, 0);
    // m_pidControllerY.setTolerance(0.5);

    m_profiledPidX = new TuneableProfiledPID("m_profiledPidX", 0.065, 0.0, 0.0, 7.0, 10.0);
    m_profiledPidX.setTolerance(0.5);
    // m_pidControllerX = new PIDController(0.045, 0, 0);
    // m_pidControllerX.setTolerance(0.5);

    m_thetaController = new PIDController(0.12, 0.0, 0.0);
    m_thetaController.enableContinuousInput(-180, 180);
    m_thetaController.setTolerance(3);

    targetReefAngle = storedState.getTargetReefAngle();
    reefAngle = LimelightSubsystem.getLLReefAngle(limelightName);
    isStrafeReached = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_profiledPidX.updatePID();
    m_profiledPidY.updatePID();

    // if (!isAngleReached) {
    //   if ((Math.abs(reefAngle) == 180.0
    //           ? Math.abs(Math.abs(m_drive.getRotation().getDegrees()) - 180)
    //           : Math.abs(m_drive.getRotation().getDegrees() - reefAngle))
    //       < 2.5) {
    //     isAngleReached = true;
    //   }
    //   rotationVal = m_thetaController.calculate(m_drive.getRotation().getDegrees(), reefAngle);
    //   m_thetaController.calculate(
    //       (MathUtil.inputModulus(m_drive.getHeadingDegrees(), -180, 180)), reefAngle);
    //   // rotationVal = MathUtil.clamp(rotationVal, -0.3, 0.3);
    //   // If it is reached, start moving towards the target
    //   ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, rotationVal);
    //   m_drive.runVelocity(speeds);
    // }

    if (LimelightHelpers.getTV(limelightName)) {
      if (isAngleReached) {
        // rotationVal =
        //     m_thetaController.calculate(m_drive.getRotation().getDegrees(), targetReefAngle);

        rotationVal =
            m_thetaController.calculate(
                m_drive.getHeadingDegrees(), Math.round(m_drive.getHeadingDegrees() / 60) * 60);

        xTrans = m_profiledPidX.calculate(LimelightHelpers.getTX(limelightName));

        xTrans = MathUtil.clamp(xTrans, -10, 10);

        yTrans = m_profiledPidY.calculate(LimelightHelpers.getTA(limelightName) - desiredArea);
        yTrans = MathUtil.clamp(yTrans, -10, 10);

        // If x-values within tolerance, start superstructuring
        if (isStrafeReached) {
          arm.setGoalVoid(currentState);
        } else {
          if (Math.abs(m_profiledPidX.calculate(LimelightHelpers.getTX(limelightName))) < 0.25) {
            isStrafeReached = true;
          }
        }
      }

      // Do all the speeds stuff

      if (isAngleReached) {
        MathUtil.clamp(rotationVal, -0.75, 0.75);
      }
      if (isStrafeReached) {
        MathUtil.clamp(xTrans, -1.0, 1.0);
      }
      // if (LimelightHelpers.getTA(limelightName) < 10.0) {
      //   yTrans = Math.abs(yTrans);
      // }

      yTrans = MathUtil.applyDeadband(yTrans, 0.2);
      xTrans = MathUtil.applyDeadband(xTrans, 0.2);
      rotationVal = MathUtil.applyDeadband(rotationVal, 0.05);

      ChassisSpeeds speeds = new ChassisSpeeds(-yTrans, -xTrans, rotationVal);
      SmartDashboard.putNumber("ProfiledPIDX", xTrans);
      SmartDashboard.putNumber("ProfiledPIDY", yTrans);
      SmartDashboard.putNumber("RotationValue", rotationVal);

      m_drive.runVelocity(speeds);

      if (m_profiledPidX.atSetpoint() && m_profiledPidY.atSetpoint() && count > 25) {
        m_end = true;
      } else {
        m_end = false;
      }
    }
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
