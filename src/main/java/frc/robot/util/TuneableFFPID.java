// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class TuneableFFPID extends PIDController {

  // Instance name for tagging log values
  String m_name;

  // Tunable numbers
  private LoggedTunableNumber m_kP, m_kI, m_kD, m_kFF;

  private double m_kFeedForward;

  public TuneableFFPID(String name, double kP, double kI, double kD, double kFF) {
    super(kP, kI, kD);

    m_name = name;
    m_kFeedForward = kFF;

    // Tunable numbers for PID and motion gain constants
    m_kP = new LoggedTunableNumber(m_name + "/kP", kP);
    m_kI = new LoggedTunableNumber(m_name + "/kI", kI);
    m_kD = new LoggedTunableNumber(m_name + "/kD", kD);

    m_kFF = new LoggedTunableNumber(m_name + "/FF", kFF);
  }

  public void updatePID() {
    // If changed, update controller constants from Tuneable Numbers
    if (m_kP.hasChanged(hashCode()) || m_kI.hasChanged(hashCode()) || m_kD.hasChanged(hashCode())) {
      this.setPID(m_kP.get(), m_kI.get(), m_kD.get());
    }

    if (m_kFF.hasChanged(hashCode())) {
      m_kFeedForward = m_kFF.get();
    }
  }

  public double calculateWithFF(double measurement) {
    return this.calculate(measurement) + m_kFeedForward;
  }

  public double calculateWithFF(double measurement, double setpoint) {
    return this.calculate(measurement, setpoint) + m_kFeedForward;
  }
}
