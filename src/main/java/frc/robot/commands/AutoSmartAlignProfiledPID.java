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
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.MachineStates.BotState;
import frc.robot.util.TuneableProfiledPID;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoSmartAlignProfiledPID extends Command {
  /** Creates a new SmartAlign. */
  Drive m_drive;

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

  String llName;

  public AutoSmartAlignProfiledPID(Drive d, String llName) {
    this.m_drive = d;
    this.llName = llName;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    limelightName = llName;

    m_end = false;

    m_profiledPidY = new TuneableProfiledPID("m_profiledPidY", 0.14, 0.0, 0.005, 3.0, 3);
    m_profiledPidY.setTolerance(0.5);

    m_profiledPidX = new TuneableProfiledPID("m_profiledPidX", 0.04, 0.0, 0.001, 3.0, 3.0);
    m_profiledPidX.setTolerance(0.5);

    m_thetaController = new PIDController(0.12, 0.0, 0.0);
    m_thetaController.enableContinuousInput(-180, 180);
    m_thetaController.setTolerance(3);

    isStrafeReached = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_profiledPidX.updatePID();
    m_profiledPidY.updatePID();

    if (LimelightHelpers.getTV(limelightName)) {
      if (isAngleReached) {

        rotationVal =
            m_thetaController.calculate(
                m_drive.getHeadingDegrees(), Math.round(m_drive.getHeadingDegrees() / 60) * 60);

        xTrans = m_profiledPidX.calculate(LimelightHelpers.getTX(limelightName));

        xTrans = MathUtil.clamp(xTrans, -10, 10);

        yTrans = m_profiledPidY.calculate(LimelightHelpers.getTA(limelightName) - desiredArea);
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

        if (isStrafeReached) {
          // arm.setGoalVoid(currentState, true);
        } else {
          if (xTrans < 0.1) {
            isStrafeReached = true;
          }
        }
      }

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
