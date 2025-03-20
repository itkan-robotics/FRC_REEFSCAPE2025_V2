// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.LimelightSubsystem;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class LoggingUtil {

  public static void logMotor(String key, TalonFX motor) {
    Logger.recordOutput(key + "/Setpoint", motor.getClosedLoopReference().getValue());
    Logger.recordOutput(key + "/Position", motor.getPosition().getValue());
    Logger.recordOutput(key + "/Velocity", motor.getVelocity().getValue());
    Logger.recordOutput(key + "/Acceleration", motor.getAcceleration().getValue());
    Logger.recordOutput(key + "/DutyCycle", motor.getDutyCycle().getValue());
    Logger.recordOutput(key + "/Voltage", motor.getMotorVoltage().getValue());
    Logger.recordOutput(key + "/PIDReference", motor.getClosedLoopOutput().getValue());
    Logger.recordOutput(key + "/TempºC", motor.getDeviceTemp().getValue());
  }

  public static void logCamera(String limelightName) {
    Logger.recordOutput(limelightName + "/TV", LimelightHelpers.getTV(limelightName));
    Logger.recordOutput(limelightName + "/TX", LimelightHelpers.getTX(limelightName));
    Logger.recordOutput(limelightName + "/TY", LimelightHelpers.getTY(limelightName));
    Logger.recordOutput(limelightName + "/TA", LimelightHelpers.getTA(limelightName));
    Logger.recordOutput(
        limelightName + "/Latency",
        LimelightHelpers.getLatency_Capture(limelightName)
            + LimelightHelpers.getLatency_Pipeline(limelightName));
    Logger.recordOutput(
        limelightName + "/Pipeline", LimelightHelpers.getCurrentPipelineIndex(limelightName));
    Logger.recordOutput(
        limelightName + "/Primary ID", LimelightHelpers.getFiducialID(limelightName));
    Logger.recordOutput(
        limelightName + "/Ambiguity",
        LimelightSubsystem.getPrimaryFiducial(LimelightHelpers.getRawFiducials(limelightName))
            .ambiguity);
    Logger.recordOutput(
        limelightName + "/DistToPrimaryID",
        LimelightSubsystem.getPrimaryFiducial(LimelightHelpers.getRawFiducials(limelightName))
            .distToCamera);
    Logger.recordOutput(
        limelightName + "/BotPose",
        (LimelightHelpers.getBotPose2d_wpiBlue(limelightName) != null)
            ? LimelightHelpers.getBotPose2d_wpiBlue(limelightName)
            : new Pose2d(0, 0, Rotation2d.kZero));
  }
}
