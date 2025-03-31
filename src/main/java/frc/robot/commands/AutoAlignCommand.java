// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignCommand extends Command {
  Drive m_drive;
  LimelightSubsystem m_limelight;
  private String limelightName;
  PIDController m_pidControllerY, m_pidControllerX;
  boolean m_end;
  double xTrans = 0.0;
  double yTrans = 0.0;
  double count = 0;
  double angle = 0;
  double rotationVal;
  private PIDController m_thetaController;

  public AutoAlignCommand(
      Drive drive, LimelightSubsystem limelight, double angle, String limelightName) {
    this.m_drive = drive;
    this.m_limelight = limelight;
    this.limelightName = limelightName;
    this.angle = angle;
    // addRequirements(this.m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidControllerY = new PIDController(0.05, 0, 0);
    m_pidControllerY.setTolerance(0.5);

    m_pidControllerX = new PIDController(0.05, 0, 0);
    m_pidControllerX.setTolerance(0.5);

    m_thetaController = new PIDController(0.03, 0, 0);
    m_pidControllerX.setTolerance(3);

    m_thetaController.enableContinuousInput(-180, 180);

    m_end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_limelight.hasTarget(limelightName)) {
      m_end = true;
    }

    m_thetaController.setSetpoint(angle);

    // rotationVal = m_thetaController.calculate(m_drive.getRotation().getDegrees(), angle);
    //     m_thetaController.calculate(
    //         (MathUtil.inputModulus(m_drive.getHeadingDegrees(), -180, 180)), angle);
    // rotationVal = MathUtil.clamp(rotationVal, -0.3, 0.3);

    m_pidControllerX.setSetpoint(0);
    xTrans = m_pidControllerX.calculate(m_limelight.getX(limelightName));
    xTrans = MathUtil.clamp(xTrans, -5, 5);

    m_pidControllerY.setSetpoint(19);
    yTrans = m_pidControllerY.calculate(m_limelight.getArea(limelightName));
    yTrans = MathUtil.clamp(yTrans, -5, 5);

    // y x theta
    // ChassisSpeeds speeds =
    //     ChassisSpeeds.fromFieldRelativeSpeeds(yTrans, xTrans, 0, m_drive.getRotation());
    ChassisSpeeds speeds = new ChassisSpeeds(-yTrans, -xTrans, 0);

    m_drive.runVelocity(speeds);

    if (m_pidControllerX.atSetpoint() && m_pidControllerY.atSetpoint() && count > 25) {
      m_end = true;
    } else {
      m_end = false;
    }

    count++;
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
