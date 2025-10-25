package frc.robot.util;

import frc.robot.subsystems.Superstructure;
import java.util.*;

public class Graph<T> {
  HashMap<T, List<Pair<T>>> graph;

  public Graph(T[] arr) {
    graph = new HashMap<>();
    for (T t : arr) {
      graph.put(t, new ArrayList<Pair<T>>());
    }
  }

  public void add(T startState, T endState, double weight) {
    if (graph.containsKey(startState)) {
      List<Pair<T>> edges = graph.get(startState);
      edges.add(new Pair<T>(endState, weight));
      graph.put(startState, edges);
    } else {
      throw new IllegalArgumentException();
    }
  }

  public void addBoth(T startState, T endState, double transitionCost) {
    if (graph.containsKey(startState) && graph.containsKey(endState)) {
      add(startState, endState, transitionCost);
      add(endState, startState, transitionCost);
    } else {
      throw new IllegalArgumentException();
    }
  }

  public List<Pair<T>> getAdj(Superstructure.State state) {
    return graph.get(state);
  }
}
