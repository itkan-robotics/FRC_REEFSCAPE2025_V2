package frc.robot.util;

import frc.robot.subsystems.*;

public class Pair<T, A> {
  public T state;
  public A cost;

  public Pair(T state, A cost) {
    this.state = state;
    this.cost = cost;
  }
}
