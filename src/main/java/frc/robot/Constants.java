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

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  public static enum TagOffsets {
    LEFT_BRANCH(1),
    CENTER(2),
    ALGAE(2),
    RIGHT_BRANCH(3);

    private final int pipeline;

    TagOffsets(int pipeline) {
      this.pipeline = pipeline;
    }

    public int getPipeline() {
      return pipeline;
    }
  };

  public static final double translationalAutoP = 3.4;
  public static final double rotationalAutoP = 6.1;

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

  public static final int LEFT_CORAL_INATKE_MOTOR_PORT = 13; // 1 is a place holder
  public static final int RIGHT_CORAL_INATKE_MOTOR_PORT = 12; // 1 is a place holder
  public static final int CORAL_OUTTAKE_MOTOR_PORT = 9;

  public static class ElevatorConstants {
    // Only Left motor being used
    public static final int LEFT_ELEVATOR_MOTOR_PORT = 11;
    // public static final int RIGHT_ELEVATOR_MOTOR_PORT = 11;
    public static final double kElevatorGearRatio = 1.0;
    public static final double kRotationsToInchesRatio = 1.0;
    public static final double elevator_kP = 0.0;
    public static final double elevator_kD = 0.0;
    public static final boolean LEFT_ELEVATOR_IS_INVERTED = false;
    public static final boolean RIGHT_ELEVATOR_IS_INVERTED = false;
  }

  public static class LimelightConstants {
    public static final double VELOCITY_DEADBAND = 0.025;

    public static final double MAX_AREA = 8.0; // Must be tuned once field is built
    public static final double MIN_AREA = 0.01;
    public static final double MAX_KP = 0.12; // Must be tuned once field is built
    public static final double MIN_KP = 0.015; // Must be tuned once field is built
    public static final double ALIGN_KS = 0.05;

    public static final double TURN_KP = 0.23;
    public static final double TURN_KD = 0.05;

    public static final double CENTERING_KP = 1.695;
    public static final double CENTERING_KD = 0.001;
    public static final double BRANCH_OFFSET = 10.0; // Must be tuned once field is built
  }
}
