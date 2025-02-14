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
import edu.wpi.first.math.util.Units;
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
  public static final int INTAKE_MOTOR_PORT = 17;

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
    ALGAELOLLIPOP(35.5, 5),
    ALGAEGROUND(44, 6),
    CORALINTAKE(0.0, 0.0),
    LEVELONE(20.0, 10.0),
    LEVELTWO(20.0, 15.0),
    LEVELTHREE(15.0, 23.0),
    LEVERLFOUR(9.0, 39.0),
    HOME(17.5, 0),
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
    /**
     * The desired offset from the limelight to the reef in meters (negative since we want to be
     * farther away, not closer up)
     *
     * <p>Last Updated by Abdullah Khaled, 2/9/2025
     */
    public static final double kReefOffset = -0.38;

    /**
     * Enum to store the offsets for the left and right branches on the reef. Includes value of the
     * offset in inches with methods to get the offset in various units.
     *
     * <p>Last Updated by Abdullah Khaled, 2/9/2025
     */
    public static enum TagOffsets {
      LEFT_BRANCH(-8.0),
      CENTER(0.0),
      ALGAE(0.0),
      RIGHT_BRANCH(8.0);

      private final double horizontalOffsetInches;

      TagOffsets(double offsetInches) {
        this.horizontalOffsetInches = offsetInches;
      }

      /**
       * Returns the limelight offset in inches
       *
       * <p>Last Updated by Abdullah Khaled, 2/9/2025
       */
      public double getHorizontalOffsetInches() {
        return horizontalOffsetInches;
      }

      /**
       * Returns the limelight offset in meters
       *
       * <p>Last Updated by Abdullah Khaled, 2/9/2025
       */
      public double getHorizontalOffsetMeters() {
        return Units.inchesToMeters(horizontalOffsetInches);
      }
    };
  }

  // PathPlanner config constants (wrong values)
  public static final double ROBOT_MASS_KG = 18.143;
  public static final double ROBOT_MOI = 1.965;
  public static final double WHEEL_COF = 1.2;
  public static final double translationalAutoP = 6.0;
  public static final double rotationalAutoP = 15.0;

  // values from Team Spectrum 3847’s X-Ray robot from 2023
  public static final Vector<N3> VISION_STDS = VecBuilder.fill(5, 5, 500);
}
