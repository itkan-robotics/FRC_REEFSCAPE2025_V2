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

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
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

  public static class ElevatorConstants {
    // Only Left motor being used
    public static final int LEFT_ELEVATOR_MOTOR_PORT = 13;
    public static final int LEFT_CORAL_INATKE_MOTOR_PORT = 1; // 1 is a place holder
    public static final int RIGHT_CORAL_INATKE_MOTOR_PORT = 12; // 1 is a place holder
    public static final int CORAL_OUTTAKE_MOTOR_PORT = 15;
    public static final int RIGHT_PIVOT_MOTOR_PORT = 10;
    public static final int LEFT_PIVOT_MOTOR_PORT = 9;
    public static final double kElevatorGearRatio = 1.0;
    public static final double kRotationsToInchesRatio = 1.0;
    public static final double elevator_kP = 0.0;
    public static final double elevator_kD = 0.0;
    public static final boolean LEFT_ELEVATOR_IS_INVERTED = true;
    public static final boolean RIGHT_ELEVATOR_IS_INVERTED = false;
  }

  public static class LimelightConstants {
    public static final double MAX_AREA = 18.5;
    public static final double SPEED_KP = 0.0;
    public static final double SPEED_KI = 0.0;
    public static final double SPEED_KD = 0.0;
    public static final double SPEED_MAXVEL = 0.0;
    public static final double SPEED_MAXACCEL = 0.0;
    public static final TrapezoidProfile.Constraints SPEED_CONTRAINTS =
        new TrapezoidProfile.Constraints(SPEED_MAXVEL, SPEED_MAXACCEL);
    public static final double ALIGN_KP = 0.0;
    public static final double ALIGN_KI = 0.0;
    public static final double ALIGN_KD = 0.0;
  }
}
