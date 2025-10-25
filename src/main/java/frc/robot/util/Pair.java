package frc.robot.util;

import frc.robot.subsystems.*;

public class Pair<T> {
  public T state;
  public double cost;

  public Pair(T state, double cost) {
    this.state = state;
    this.cost = cost;
  }
}
