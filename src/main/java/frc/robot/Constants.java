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

  public static final int SCORE_MOTOR_PORT = 12; // 1 is a place holder
  public static final int INTAKE_MOTOR_PORT = 0;

  public static class PivotConstants {
    public static final int RIGHT_PIVOT_MOTOR_PORT = 10;
    public static final int LEFT_PIVOT_MOTOR_PORT = 9;
    public static final double PIVOT_KP = 17.0;
    public static final double PIVOT_KS = 4.0;
    public static final double PIVOT_CRUISE_VELOCITY = 65.0;
    public static final double PIVOT_ACCELERATION = 80.0;
  }

  public static class ElevatorConstants {
    // Only Left motor being used
    public static final int LEFT_ELEVATOR_MOTOR_PORT = 13;

    public static final double kElevatorGearRatio = 1.0;
    public static final double kRotationsToInchesRatio = 1.0;
    public static final double ELEVATOR_KP = 50.0;
    public static final double ELEVATOR_KS = 7.5;
    public static final double ELEVATOR_CRUISE_VELOCITY = 60.0;
    public static final double ELEVATOR_ACCELERATION = 250.0;
    public static final double ELEVATOR_JERK = 750.0;
    public static final boolean LEFT_ELEVATOR_IS_INVERTED = true;
    public static final boolean RIGHT_ELEVATOR_IS_INVERTED = false;
  }

  public static enum CoralPos {
    BARGE(0.0, 0.0),
    ALGAEINTAKEONE(23, 10),
    ALGAEINTAKETWO(18.5, 20),
    ALGAELOLLIPOP(36, 5),
    ALGAEGROUND(44, 6),
    CORALINTAKE(0.0, 0.0),
    LEVELONE(20.0, 10.0),
    LEVELTWO(20.0, 15.0),
    LEVELTHREE(15.0, 23.0),
    LEVERLFOUR(9.0, 39.0),
    HOME(5.0, 0),
    TEST(13.0, 0),
    CLIMB(49, 8);

    private final double pivotSetpoint;
    private final double elevatorSetpoint;

    CoralPos(double pivotSetpoint, double elevatorSetpoint) {
      this.pivotSetpoint = pivotSetpoint;
      this.elevatorSetpoint = elevatorSetpoint;
    }

    public double getPivotSetpoint() {
      return pivotSetpoint;
    }

    public double getElevatorSetpoint() {
      return elevatorSetpoint;
    }
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
