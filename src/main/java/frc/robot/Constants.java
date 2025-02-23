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

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final boolean kIsTuningMode = true;
  public static final boolean tuningMode = true;

  // PathPlanner config constants (wrong values)
  public static final double ROBOT_MASS_KG = 18.143;
  public static final double ROBOT_MOI = 1.965;
  public static final double WHEEL_COF = 1.2;
  public static final double translationalAutoP = 6.0;
  public static final double rotationalAutoP = 15.0;

  // Values from Team Spectrum 3847’s X-Ray robot from 2023
  public static final Vector<N3> VISION_STDS = VecBuilder.fill(5, 5, 500);

  public static final int SCORE_MOTOR_PORT = 12;
  public static final int INTAKE_MOTOR_PORT = 17;

  public static class ActuatorConstants {
    public static final int RIGHT_ACTUATOR_MOTOR_PORT = 10;
    public static final int LEFT_ACTUATOR_MOTOR_PORT = 9;
    public static final double ACTUATOR_KP = 28.0;
    public static final double ACTUATOR_KS = 4.0;
    public static final double ACTUATOR_CRUISE_VELOCITY = 65.0;
    public static final double ACTUATOR_ACCELERATION = 80.0;
  }

  public static class ElevatorConstants {
    public static final int ELEVATOR_MOTOR_PORT = 13;

    public static final double ELEVATOR_KP = 50.0;
    public static final double ELEVATOR_KS = 7.5;
    public static final double ELEVATOR_KG = 0.0;
    public static final double ELEVATOR_CRUISE_VELOCITY = 60.0;
    public static final double ELEVATOR_ACCELERATION = 250.0;
    public static final double ELEVATOR_JERK = 750.0;
  }

  /** The different elevator and pivot states our robot can do, all in one enum! */
  public static enum BotState {
    RESET(5.0, 0.0),
    CORALINTAKE(6.0, 0.0),
    L1(28.0, 3.0),
    L2(15.5, 17),
    L3(14.5, 25),
    L4(11.5, 38),
    HOME(19.0, 0),
    LOWALGAE(18, 10),
    HIGHALGAE(15, 18.5),
    GROUNDALGAE(43, 6), // 53
    BARGE(5, 39.9),
    PROCESSOR(40, 5); // TO-DO: Test and tune

    private final double pivotSetpoint;
    private final double elevatorSetpoint;

    BotState(double actuatorSetpoint, double elevatorSetpoint) {
      this.pivotSetpoint = actuatorSetpoint;
      this.elevatorSetpoint = elevatorSetpoint;
    }

    public double getActuatorSetpoint() {
      return pivotSetpoint;
    }

    public double getElevatorSetpoint() {
      return elevatorSetpoint;
    }
  }

  public static String getBotStateAsString(BotState state) {
    switch (state) {
      case BARGE:
        return "BARGE";
      case CORALINTAKE:
        return "CORALINTAKE";
      case GROUNDALGAE:
        return "GROUNDALGAE";
      case HIGHALGAE:
        return "HIGHALGAE";
      case HOME:
        return "HOME";
      case L1:
        return "L1";
      case L2:
        return "L2";
      case L3:
        return "L3";
      case L4:
        return "L4";
      case LOWALGAE:
        return "LOWALGAE";
      case PROCESSOR:
        return "PROCESSOR";
      case RESET:
        return "RESET";
      default:
        return "HOW DID WE GET HERE?";
    }
  }

  public static class LimelightConstants {
    /**
     * The desired offset from the limelight to the reef in meters (negative since we want to be
     * farther away, not closer up)
     */
    public static final String limelightName = "limelight";

    public static final double kReefOffset = -0.35;

    public static final int LEFT_BRANCH_PIPELINE = 1;
    public static final int RIGHT_BRANCH_PIPELINE = 2;
    public static final int CENTER_PIPELINE = 0;
    public static final int DEFAULT_PIPELINE = 0;

    public static final double kLeftBranchXOffset = -0.33 / 2;
    public static final double kRightBranchXOffset = 0.33 / 2;
    public static final double kDefaultXOffset = 0.0;

    public static final double VELOCITY_DEADBAND = 0.025;

    public static final double MAX_AREA = 8.0; // Must be tuned once field is built
    public static final double MIN_AREA = 0.01;
    public static final double MAX_KP = 0.12; // Formerly // Must be tuned once field is built
    public static final double MIN_KP = 0.015; // Must be tuned once field is built
    public static final double ALIGN_KS = 0.05;

    public static final double TURN_KP = 0.23;
    public static final double TURN_KD = 0.05;

    public static final double CENTERING_KP = 1.695;
    public static final double CENTERING_KD = 0.001;
    public static final double BRANCH_OFFSET = 10.0; // Must be tuned once field is built
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
