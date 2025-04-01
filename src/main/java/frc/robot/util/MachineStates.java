// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class MachineStates {
  public static class BotState {
    private String name;
    private double shoulderSetpoint;
    private double extensionSetpoint;
    private double wristSetpoint;
    private int id;

    // Constructor
    public BotState(
        String name,
        double shoulderSetpoint,
        double extensionSetpoint,
        double wristSetpoint,
        int id) {
      this.name = name;
      this.extensionSetpoint = extensionSetpoint;
      this.shoulderSetpoint = shoulderSetpoint;
      this.wristSetpoint = wristSetpoint;
      this.id = id;
    }

    // Getters
    public String getName() {
      return name;
    }

    public double getShoulderSetpoint() {
      return shoulderSetpoint;
    }

    public double getExtensionSetpoint() {
      return extensionSetpoint;
    }

    public double getWristSetpoint() {
      return wristSetpoint;
    }

    public int getId() {
      return id;
    }

    @Override
    public String toString() {
      return "BotState{"
          + "name='"
          + name
          + '\''
          + ", armSetpoint="
          + extensionSetpoint
          + ", pivotSetpoint="
          + shoulderSetpoint
          + ", id="
          + id
          + '}';
    }
  }

  public static final BotState HOME =
      new BotState("HOME", 0.05, 1.5, 0.3, 0); // 0.125 for shoulder home 0.15 for wrist home

  // Coral Handling
  public static final BotState L1 = new BotState("L1", -0.13, -1.5, 0.3, 1);
  public static final BotState L2 = new BotState("L2", 0.19, 5, 0.5, 2);
  public static final BotState L3 = new BotState("L3", 0.155, 4.5, 0.37, 3);
  public static final BotState L4 = new BotState("L4", 0.13, 20, 0.49, 4); // 20.5
  public static final BotState INTAKE = new BotState("INTAKE", -0.16, -1.95, 0.11, 5); // 0

  // Algae Handling
  public static final BotState LOWALGAE = new BotState("LOWALGAE", 0.18, 3.0, 0.35, 6);
  public static final BotState HIGHALGAE = new BotState("HIGHALGAE", 0.18, 3.0, 0.2, 7);
  public static final BotState PROCESSOR = new BotState("PROCESSOR", 0.0, 0.0, 0.0, 8);
  public static final BotState NET = new BotState("NET", 0.0, 0.0, 0.0, 9);

  // MISC
  public static final BotState PRECLIMB = new BotState("PRECLIMB", 0.14, 3.5, 0.0, 10);
  public static final BotState CLIMB = new BotState("CLIMB", -0.16, -1.95, 0.13, 11);
  public static final BotState RESET = new BotState("RESET", 0.0, 0.0, 0.0, -1);
  public static final BotState INTAKEAUTO = new BotState("INTAKEAUTO", 0.0, 0.0, 0.0, 20);

  // public static final BotState Intake = new BotState("INTAKE", 0.0, 0.0, 0.0, 12);

  public static final BotState[] PossibleStates = {
    HOME, L1, L2, L3, L4, INTAKE, LOWALGAE, HIGHALGAE, PROCESSOR, NET, PRECLIMB, CLIMB
  };
}
