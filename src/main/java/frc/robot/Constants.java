// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file). 9128 Constants
 */
public final class Constants {

  public static final boolean tuningMode = true;

  // PathPlanner config constants (wrong values)
  public static final double ROBOT_MASS_KG = 61.23;
  public static final double ROBOT_MOI = 6.429;
  public static final double WHEEL_COF = 1.2;
  public static final double translationalAutoP = 5.0;
  public static final double rotationalAutoP = 10.0;

  // Values from Team Spectrum 3847’s X-Ray robot from 2023
  public static final Vector<N3> VISION_STDS = VecBuilder.fill(5, 5, 500);

  public static final int END_EFFECTOR_MOTOR_PORT = 15;

  public static class ArmConstants {
    public static class ShoulderConstants {
      public static final int LEFT_SHOULDER_MOTOR_PORT = 9;
      public static final int SHOULDER_MOTOR_PORT = 10;
      public static final int SHOULDER_MOTOR_PORT_C = 11;

      public static final double SHOULDER_KP = 0.0;
      public static final double SHOULDER_KS = 0.0;
      public static final double SHOULDER_KG = 0.0;
      public static final double SHOULDER_KV = 0.0;
      public static final double SHOULDER_CRUISE_VELOCITY = 0.0;
      public static final double SHOULDER_ACCELERATION = 0.0;
      public static final double SHOULDER_JERK = 0.0;

      public static final double MIN_SHOULDER_ROTATION_POS = 0.0;
      public static final double MAX_SHOULDER_ROTATION_POS = 0.0;
    }

    public static class ExtensionConstants {
      public static final int EXTENSION_MOTOR_PORT_A = 12;
      public static final int EXTENSION_MOTOR_PORT_B = 13;

      public static final double EXTENSION_KP = 0.0;
      public static final double EXTENSION_KS = 0.0;
      public static final double EXTENSION_KG = 0.0;
      public static final double EXTENSION_KV = 0.0;
      public static final double EXTENSION_CRUISE_VELOCITY = 60.0;
      // public static final double EXTENSION_ACCELERATION = 0.0;
      // public static final double EXTENSION_JERK = 0.0;

      public static final double MIN_EXTENSION_POS = 0.05;
      public static final double MAX_EXTENSION_POS = 39.95;
    }

    public static class WristConstants {
      public static final int WRIST_MOTOR_PORT_A = 14;

      public static final double WRIST_KP = 0.0;
      public static final double WRIST_KS = 0.0;
      public static final double WRIST_KG = 0.0;
      public static final double WRIST_KV = 0.0;
      public static final double WRIST_CRUISE_VELOCITY = 0.0;
      public static final double WRIST_ACCELERATION = 0.0;
      public static final double WRIST_JERK = 0.0;

      public static final double MIN_WRIST_ROTATION_POS = 0.0;
      public static final double MAX_WRIST_ROTATION_POS = 0.0;
    }
  }

  public static class ClimbConstants {
    public static final int CLIMB_MOTOR_PORT = 15;
  }

  public static class LimelightConstants {

    public static final String singleLimelightName = "limelight";
    public static final String leftLimelightName = "limelight-left";
    public static final String rightLimelightName = "limelight-right";
    public static final boolean multipleLimelights = true;

    /**
     * The desired offset from the limelight to the reef in meters (negative since we want to be
     * farther away, not closer up)
     */
    public static final double kReefOffset = -0.35;

    public static final int LEFT_BRANCH_PIPELINE = 1;
    public static final int RIGHT_BRANCH_PIPELINE = 2;

    public static final double kLeftBranchXOffset = -0.33 / 2;
    public static final double kRightBranchXOffset = 0.33 / 2;
    public static final double kDefaultXOffset = 0.0;

    public static final double VELOCITY_DEADBAND = 0.025;

    public static final double MAX_AREA = 15.0; // Must be tuned once field is built
    public static final double MIN_AREA = 0.01;
    public static final LoggedTunableNumber DRIVE_KP =
        new LoggedTunableNumber(
            "interpolation/Drive_kP", 0.055); // Formerly 0.15// Must be tuned once field is built
    public static final LoggedTunableNumber DRIVE_KD =
        new LoggedTunableNumber(
            "interpolation/Drive_kP", 0.055); // Formerly 0.15// Must be tuned once field is built
    // Must be tuned once field is built

    public static final double TURN_KP = 0.15;
    public static final double TURN_KD = 0.00;

    public static final double ALIGN_KS = 0.05;
    public static final double ALIGN_KP = 2.5;
    public static final double ALIGN_KD = 0.005;
    public static final double BRANCH_OFFSET = 10.0; // Must be tuned once field is built
  }

  public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    private static final Slot0Configs steerGains =
        new Slot0Configs()
            .withKP(100)
            .withKI(0)
            .withKD(0.0)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput

    private static final Slot0Configs driveGains =
        new Slot0Configs()
            .withKP(2.2)
            .withKI(0)
            .withKD(0.0)
            .withKS(0.156571914641674)
            .withKV(0.765254407501612)
            .withKA(0.0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType =
        DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType =
        SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(80.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a
                    // relatively
                    // low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true));
    private static final TalonFXConfiguration steerInitialConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a
                    // relatively
                    // low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(25))
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true));
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("static", "./logs/example.hoot");

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.0);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5;

    private static final double kDriveGearRatio = 6.75;
    private static final double kSteerGearRatio = 25.0;
    private static final Distance kWheelRadius = Inches.of(1.953036686729617 * 0.95);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = false;

    private static final int kPigeonId = 0;

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants =
        new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        ConstantCreator =
            new SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // + rotation is counterclockwise if looking at swerve from below

    // Front Left
    private static final int kFrontLeftDriveMotorId = 1;
    private static final int kFrontLeftSteerMotorId = 2;
    private static final int kFrontLeftEncoderId = 0;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.05); // TO DO: TUNE 0.055
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(12.5);
    private static final Distance kFrontLeftYPos = Inches.of(12.5);

    // Front Right
    private static final int kFrontRightDriveMotorId = 3;
    private static final int kFrontRightSteerMotorId = 4;
    private static final int kFrontRightEncoderId = 1;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(0.89 - 0.01); // 0.83
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(12.5);
    private static final Distance kFrontRightYPos = Inches.of(-12.5);

    // Back Left
    private static final int kBackLeftDriveMotorId = 5;
    private static final int kBackLeftSteerMotorId = 6;
    private static final int kBackLeftEncoderId = 2;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.617 - 0.0035); // 0.625
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-12.5);
    private static final Distance kBackLeftYPos = Inches.of(12.5);

    // Back Right
    private static final int kBackRightDriveMotorId = 7;
    private static final int kBackRightSteerMotorId = 8;
    private static final int kBackRightEncoderId = 3;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.71 - 0.005); // 0.68 + 0.03
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-12.5);
    private static final Distance kBackRightYPos = Inches.of(-12.5);

    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontLeft =
            ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId,
                kFrontLeftDriveMotorId,
                kFrontLeftEncoderId,
                kFrontLeftEncoderOffset,
                kFrontLeftXPos,
                kFrontLeftYPos,
                kInvertLeftSide,
                kFrontLeftSteerMotorInverted,
                kFrontLeftEncoderInverted);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontRight =
            ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId,
                kFrontRightDriveMotorId,
                kFrontRightEncoderId,
                kFrontRightEncoderOffset,
                kFrontRightXPos,
                kFrontRightYPos,
                kInvertRightSide,
                kFrontRightSteerMotorInverted,
                kFrontRightEncoderInverted);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackLeft =
            ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId,
                kBackLeftDriveMotorId,
                kBackLeftEncoderId,
                kBackLeftEncoderOffset,
                kBackLeftXPos,
                kBackLeftYPos,
                kInvertLeftSide,
                kBackLeftSteerMotorInverted,
                kBackLeftEncoderInverted);
    public static final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackRight =
            ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId,
                kBackRightDriveMotorId,
                kBackRightEncoderId,
                kBackRightEncoderOffset,
                kBackRightXPos,
                kBackRightYPos,
                kInvertRightSide,
                kBackRightSteerMotorInverted,
                kBackRightEncoderInverted);

    /**
     * Creates a CommandSwerveDrivetrain instance. This should only be called once in your robot
     * program,.
     */
    //   public static CommandSwerveDrivetrain createDrivetrain() {
    //     return new CommandSwerveDrivetrain(
    //         DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    //   }

    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
      /**
       * Constructs a CTRE SwerveDrivetrain using the specified constants.
       *
       * <p>This constructs the underlying hardware devices, so users should not construct the
       * devices themselves. If they need the devices, they can access them through getters in the
       * classes.
       *
       * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
       * @param modules Constants for each specific module
       */
      public TunerSwerveDrivetrain(
          SwerveDrivetrainConstants drivetrainConstants,
          SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
      }

      /**
       * Constructs a CTRE SwerveDrivetrain using the specified constants.
       *
       * <p>This constructs the underlying hardware devices, so users should not construct the
       * devices themselves. If they need the devices, they can access them through getters in the
       * classes.
       *
       * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
       * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or
       *     set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
       * @param modules Constants for each specific module
       */
      public TunerSwerveDrivetrain(
          SwerveDrivetrainConstants drivetrainConstants,
          double odometryUpdateFrequency,
          SwerveModuleConstants<?, ?, ?>... modules) {
        super(
            TalonFX::new,
            TalonFX::new,
            CANcoder::new,
            drivetrainConstants,
            odometryUpdateFrequency,
            modules);
      }

      /**
       * Constructs a CTRE SwerveDrivetrain using the specified constants.
       *
       * <p>This constructs the underlying hardware devices, so users should not construct the
       * devices themselves. If they need the devices, they can access them through getters in the
       * classes.
       *
       * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
       * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or
       *     set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
       * @param odometryStandardDeviation The standard deviation for odometry calculation in the
       *     form [x, y, theta]ᵀ, with units in meters and radians
       * @param visionStandardDeviation The standard deviation for vision calculation in the form
       *     [x, y, theta]ᵀ, with units in meters and radians
       * @param modules Constants for each specific module
       */
      public TunerSwerveDrivetrain(
          SwerveDrivetrainConstants drivetrainConstants,
          double odometryUpdateFrequency,
          Matrix<N3, N1> odometryStandardDeviation,
          Matrix<N3, N1> visionStandardDeviation,
          SwerveModuleConstants<?, ?, ?>... modules) {
        super(
            TalonFX::new,
            TalonFX::new,
            CANcoder::new,
            drivetrainConstants,
            odometryUpdateFrequency,
            odometryStandardDeviation,
            visionStandardDeviation,
            modules);
      }
    }
  }

  public class FieldConstants {

    public static Pose2d transformAtAngle(Pose2d pose, double offset) {
      double poseX = pose.getX();
      double poseY = pose.getY();
      poseX += Math.sin(pose.getRotation().getRadians()) * offset;
      poseY += Math.cos(pose.getRotation().getRadians()) * offset;
      return new Pose2d(poseX, poseY, pose.getRotation());
    }

    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);
    public static final double startingLineX =
        Units.inchesToMeters(299.438); // Measured from the inside of starting
    // line

    public static class Processor {
      public static final Pose2d centerFace =
          new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
    }

    public static class Barge {
      public static final Translation2d farCage =
          new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
      public static final Translation2d middleCage =
          new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
      public static final Translation2d closeCage =
          new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

      // Measured from floor to bottom of cage
      public static final double deepHeight = Units.inchesToMeters(3.125);
      public static final double shallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {
      public static final Pose2d leftCenterFace =
          new Pose2d(
              Units.inchesToMeters(33.526),
              Units.inchesToMeters(291.176),
              Rotation2d.fromDegrees(90 - 144.011));
      public static final Pose2d rightCenterFace =
          new Pose2d(
              Units.inchesToMeters(33.526),
              Units.inchesToMeters(25.824),
              Rotation2d.fromDegrees(144.011 - 90));
    }

    public static class Reef {
      public static final Translation2d center =
          new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
      public static final double faceToZoneLine =
          Units.inchesToMeters(12); // Side of the reef to the inside of the
      // reef zone line

      public static final Pose2d[] centerFaces =
          new Pose2d[6]; // Starting facing the driver station in clockwise
      // order
      public static final List<Map<ReefHeight, Pose3d>> branchPositions =
          new ArrayList<>(); // Starting at the right
      // branch facing the
      // driver station in
      // clockwise

      static {
        // Initialize faces
        centerFaces[0] =
            new Pose2d(
                Units.inchesToMeters(144.003),
                Units.inchesToMeters(158.500),
                Rotation2d.fromDegrees(180));
        centerFaces[1] =
            new Pose2d(
                Units.inchesToMeters(160.373),
                Units.inchesToMeters(186.857),
                Rotation2d.fromDegrees(120));
        centerFaces[2] =
            new Pose2d(
                Units.inchesToMeters(193.116),
                Units.inchesToMeters(186.858),
                Rotation2d.fromDegrees(60));
        centerFaces[3] =
            new Pose2d(
                Units.inchesToMeters(209.489),
                Units.inchesToMeters(158.502),
                Rotation2d.fromDegrees(0));
        centerFaces[4] =
            new Pose2d(
                Units.inchesToMeters(193.118),
                Units.inchesToMeters(130.145),
                Rotation2d.fromDegrees(-60));
        centerFaces[5] =
            new Pose2d(
                Units.inchesToMeters(160.375),
                Units.inchesToMeters(130.144),
                Rotation2d.fromDegrees(-120));

        // Initialize branch positions
        for (int face = 0; face < 6; face++) {
          Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
          Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
          for (var level : ReefHeight.values()) {
            Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
            double adjustX = Units.inchesToMeters(30.738);
            double adjustY = Units.inchesToMeters(6.469);

            fillRight.put(
                level,
                new Pose3d(
                    new Translation3d(
                        poseDirection
                            .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                            .getX(),
                        poseDirection
                            .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                            .getY(),
                        level.height),
                    new Rotation3d(
                        0,
                        Units.degreesToRadians(level.pitch),
                        poseDirection.getRotation().getRadians())));
            fillLeft.put(
                level,
                new Pose3d(
                    new Translation3d(
                        poseDirection
                            .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                            .getX(),
                        poseDirection
                            .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                            .getY(),
                        level.height),
                    new Rotation3d(
                        0,
                        Units.degreesToRadians(level.pitch),
                        poseDirection.getRotation().getRadians())));
          }
          branchPositions.add(fillRight);
          branchPositions.add(fillLeft);
        }
      }
    }

    public static class StagingPositions {
      // Measured from the center of the ice cream
      public static final Pose2d leftLollipop =
          new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
      public static final Pose2d middleLollipop =
          new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
      public static final Pose2d rightLollipop =
          new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
    }

    public enum ReefHeight {
      L4(Units.inchesToMeters(72), -90),
      L3(Units.inchesToMeters(47.625), -35),
      L2(Units.inchesToMeters(31.875), -35),
      L1(Units.inchesToMeters(18), 0);

      ReefHeight(double height, double pitch) {
        this.height = height;
        this.pitch = pitch; // in degrees
      }

      public final double height;
      public final double pitch;
    }

    public static Pose2d getNearestReefFace(Pose2d currentPose) {
      return currentPose.nearest(List.of(FieldConstants.Reef.centerFaces));
    }

    public enum ReefSide {
      LEFT,
      RIGHT
    }

    public static Pose2d getNearestReefBranch(Pose2d currentPose, ReefSide side) {
      return FieldConstants.Reef.branchPositions
          .get(
              List.of(FieldConstants.Reef.centerFaces).indexOf(getNearestReefFace(currentPose)) * 2
                  + (side == ReefSide.LEFT ? 1 : 0))
          .get(FieldConstants.ReefHeight.L1)
          .toPose2d();
    }

    public static Pose2d getNearestCoralStation(Pose2d currentPose) {
      double distanceToLeftStation =
          currentPose
              .getTranslation()
              .getDistance(FieldConstants.CoralStation.leftCenterFace.getTranslation());
      double distanceToRightStation =
          currentPose
              .getTranslation()
              .getDistance(FieldConstants.CoralStation.rightCenterFace.getTranslation());

      if (distanceToLeftStation > distanceToRightStation) {
        return FieldConstants.CoralStation.rightCenterFace;
      } else {
        return FieldConstants.CoralStation.leftCenterFace;
      }
    }
  }

  // Simulation stuff we aren't using this season
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /***************************************************************************************
   * Get the current alliance as specific in the Driver Station.
   * <p> Last Updated by Abdullah Khaled, 1/17/2025
   * @return The current alliance, where red is true and blue is false
   **************************************************************************************/
  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        return true;
      } else if (alliance.get() == DriverStation.Alliance.Blue) {
        return false;
      }
    }
    return false;
  }
}
