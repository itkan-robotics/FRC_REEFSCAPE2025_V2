// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LimelightConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutoScoreSelection;
import frc.robot.util.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignTeleop extends Command {
  Drive m_drive;
  LimelightSubsystem m_limelight;
  private String limelightName;
  PIDController m_pidControllerY, m_pidControllerX;
  boolean m_end;
  double xTrans = 0.0;
  double yTrans = 0.0;
  double count = 0;
  double rotationVal;
  boolean rotationReached;
  private PIDController m_thetaController;
  AutoScoreSelection storedState;
  int tagID;
  double targetReefAngle = 0.0;
  double reefAngle = 0.0;

  public AutoAlignTeleop(
      Drive drive, LimelightSubsystem limelight, AutoScoreSelection storedState) {
    this.m_drive = drive;
    this.m_limelight = limelight;
    this.storedState = storedState;
    addRequirements(this.m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidControllerY = new PIDController(0.06, 0, 0);
    m_pidControllerY.setTolerance(0.5);

    m_pidControllerX = new PIDController(0.045, 0, 0);
    m_pidControllerX.setTolerance(0.5);

    m_thetaController = new PIDController(0.03, 0, 0);
    m_pidControllerX.setTolerance(3);

    m_thetaController.enableContinuousInput(-180, 180);

    m_end = false;
    double targetLimelightInt = storedState.getLimelightTargetPipeline();
    // SmartDashboard.putNumber("autoAlignTeleop/targetLimelightInt", targetLimelightInt);

    if (targetLimelightInt == LEFT_BRANCH_PIPELINE) {
      limelightName = LimelightConstants.rightLimelightName;
    } else if (targetLimelightInt == RIGHT_BRANCH_PIPELINE) {
      limelightName = LimelightConstants.leftLimelightName;
    }
    tagID = storedState.getTargetAprilTag();
    targetReefAngle = storedState.getTargetReefAngle();
    reefAngle = LimelightSubsystem.getLLReefAngle(limelightName);
    // SmartDashboard.putNumber("autoAlignTeleop/tagID", tagID);
    // SmartDashboard.putNumber("autoAlignTeleop/reefAngle", reefAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("autoAlignTeleop/targetReefAngle", targetReefAngle);

    // SmartDashboard.putBoolean("autoAlignTeleop/getTV", LimelightHelpers.getTV(limelightName));

    // SmartDashboard.putNumber(
    //     "autoAlignTeleop/getFiducialID", LimelightHelpers.getFiducialID(limelightName));
    // SmartDashboard.putNumber("autoAlignTeleop/tagID", tagID);
    // SmartDashboard.putNumber("autoAlignTeleop/tagIDBlue", tagID + 11);
    // && (LimelightHelpers.getFiducialID(limelightName) == tagID ||
    // LimelightHelpers.getFiducialID(limelightName) == (tagID + 11))
    if (LimelightHelpers.getTV(limelightName)) {

      // && Math.abs(m_drive.getRotation().getDegrees() - reefAngle) < 10.0
      if (!m_limelight.hasTarget(limelightName)) {
        m_end = true;
      }

      m_pidControllerX.setSetpoint(0);
      xTrans = m_pidControllerX.calculate(m_limelight.getX(limelightName));
      xTrans = MathUtil.clamp(xTrans, -5, 5);

      m_pidControllerY.setSetpoint(19);
      yTrans = m_pidControllerY.calculate(m_limelight.getArea(limelightName));
      yTrans = MathUtil.clamp(yTrans, -5, 5);

      ChassisSpeeds speeds = new ChassisSpeeds(yTrans, xTrans, 0);

      m_drive.runVelocity(speeds);

      if (m_pidControllerX.atSetpoint() && m_pidControllerY.atSetpoint() && count > 25) {
        m_end = true;
      } else {
        m_end = false;
      }

      count++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_end;
  }
}
