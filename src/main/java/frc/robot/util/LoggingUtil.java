// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AppliedRotorPolarityValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.LimelightSubsystem;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class LoggingUtil {

  public static class SimpleMotorLogger {
    private final TalonFX m;
    private String key;

    public SimpleMotorLogger(TalonFX motor, String motorName) {
      m = motor;
      key = motorName;
    }

    /**
     * Logs basic motor information including:
     *
     * <ul>
     *   <li>Device connection
     *   <li>Device ID
     *   <li>Pro License
     *   <li>Software Rotation Limits
     *   <li>Temperature
     * </ul>
     *
     * InvertedValue.Clockwise_Positive
     */
    public SimpleMotorLogger logMotorSpecs() {
      Logger.recordOutput(key + "/_isConnected", m.isConnected());
      Logger.recordOutput(key + "/_deviceID", m.getDeviceID());
      Logger.recordOutput(
          key + "/Specs/isProLicensed",
          (m.getIsProLicensed().getValueAsDouble() == 1 ? true : false));
      Logger.recordOutput(key + "/Specs/forwardLimit", m.getForwardLimit().getValueAsDouble());
      Logger.recordOutput(key + "/Specs/reverseLimit", m.getReverseLimit().getValueAsDouble());
      Logger.recordOutput(key + "/_tempF", (m.getDeviceTemp().getValueAsDouble() * 1.8) + 32);
      Logger.recordOutput(key + "/Specs/tempC", m.getDeviceTemp().getValueAsDouble());
      Logger.recordOutput(
          key + "/Specs/invertedValue",
          m.getAppliedRotorPolarity().getValue() == AppliedRotorPolarityValue.PositiveIsClockwise
              ? "PositiveIsClockwise"
              : "PositiveIsCounterClockwise");
      return this;
    }

    /** Records the position, velocity, and acceleration of the motor in radians and degrees. */
    public SimpleMotorLogger logMotorPVA() {
      // Logger.recordOutput(name + "/pos", m.getPosition().getValueAsDouble());
      Logger.recordOutput(
          key + "/PVA/pRad", Units.rotationsToRadians(m.getPosition().getValueAsDouble()));
      Logger.recordOutput(
          key + "/PVA/pDeg", Units.rotationsToDegrees(m.getVelocity().getValueAsDouble()));
      Logger.recordOutput(
          key + "/PVA/vRad", Units.rotationsToRadians(m.getVelocity().getValueAsDouble()));
      Logger.recordOutput(
          key + "/PVA/vDeg", Units.rotationsToDegrees(m.getVelocity().getValueAsDouble()));
      Logger.recordOutput(
          key + "/PVA/aRad", Units.rotationsToRadians(m.getVelocity().getValueAsDouble()));
      Logger.recordOutput(
          key + "/PVA/aDeg", Units.rotationsToDegrees(m.getVelocity().getValueAsDouble()));
      //
      return this;
    }

    /** Records the applied volts, torque current, supply current, and duty cycle of the motor */
    public SimpleMotorLogger logMotorPowerData() {
      Logger.recordOutput(key + "/_appliedVolts", m.getMotorVoltage().getValueAsDouble());
      Logger.recordOutput(key + "/Current/_torqueCurrent", m.getTorqueCurrent().getValueAsDouble());
      Logger.recordOutput(key + "/Current/_supplyCurrent", m.getSupplyCurrent().getValueAsDouble());
      Logger.recordOutput(key + "/_dutyCycle", m.getDutyCycle().getValueAsDouble());
      //
      return this;
    }

    /**
     * Log various information about the PID Controller and its outputs, including:
     *
     * <ul>
     *   <li>Overall output and output for P, I, and D
     *   <li>PID Error
     *   <li>PID Reference
     *   <li>PID Feedforward
     * </ul>
     */
    public SimpleMotorLogger logMotorPID() {
      Logger.recordOutput(key + "/PID/Error", m.getClosedLoopError().getValueAsDouble());
      Logger.recordOutput(
          key + "/PID/Feedforward", m.getClosedLoopFeedForward().getValueAsDouble());
      Logger.recordOutput(key + "/PID/Reference", m.getClosedLoopReference().getValueAsDouble());
      Logger.recordOutput(key + "/PID/Output", m.getClosedLoopOutput().getValueAsDouble());
      Logger.recordOutput(
          key + "/PID/POutput", m.getClosedLoopProportionalOutput().getValueAsDouble());
      Logger.recordOutput(
          key + "/PID/IOutput", m.getClosedLoopIntegratedOutput().getValueAsDouble());
      Logger.recordOutput(
          key + "/PID/DOutput", m.getClosedLoopDerivativeOutput().getValueAsDouble());
      //
      return this;
    }

    public void setName(String name) {
      key = name;
    }
  }

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
