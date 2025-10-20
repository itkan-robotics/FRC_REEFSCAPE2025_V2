package frc.robot.util;

import frc.robot.subsystems.Superstructure;
import java.util.*;

public class Graph<T, A> {
  HashMap<T, List<Pair<T, A>>> graph;

  public Graph(T[] arr) {
    graph = new HashMap<>();
    for (T t : arr) {
      graph.put(t, new ArrayList<Pair<T, A>>());
    }
  }

  public void add(T startState, T endState, A weight) {
    if (graph.containsKey(startState)) {
      List<Pair<T, A>> edges = graph.get(startState);
      edges.add(new Pair<T, A>(endState, weight));
      graph.put(startState, edges);
    } else {
      throw new IllegalArgumentException();
    }
  }

  public void addBoth(T startState, T endState, A transitionCost) {
    if (graph.containsKey(startState) && graph.containsKey(endState)) {
      add(startState, endState, transitionCost);
      add(endState, startState, transitionCost);
    } else {
      throw new IllegalArgumentException();
    }
  }

  public List<Pair<T, A>> getAdj(Superstructure.State state) {
    return graph.get(state);
  }
}
