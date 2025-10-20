package frc.robot.util;

public class Pair<T, A> {
  private T first;
  private A second;

  /**
   * @param first First element in the pair
   * @param second Second element in the pair
   */
  public Pair(T first, A second) {
    this.first = first;
    this.second = second;
  }

  /**
   * @return Returns the first element in the pair
   */
  public T first() {
    return first;
  }

  /**
   * @return Returns the second element in the pair
   */
  public A second() {
    return second;
  }
}
