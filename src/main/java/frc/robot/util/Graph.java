package frc.robot.util;

import java.util.*;

public class Graph<T, A> {
  HashMap<T, List<Pair<T, A>>> graph;

  /**
   * Creates a new graph with all nodes in the array
   *
   * @param arr array of nodes to be added
   */
  public Graph(T[] arr) {
    graph = new HashMap<>();
    for (T t : arr) {
      graph.put(t, new ArrayList<Pair<T, A>>());
    }
  }

  /**
   * Adds an edge from start state to end state with a weight of weight
   *
   * @param startState initial state for edge
   * @param endState final state for edge
   * @param weight weight for edge
   * @throws IllegalArgumentException if startState is not a valid node
   */
  public void add(T startState, T endState, A weight) {
    if (graph.containsKey(startState)) {
      List<Pair<T, A>> edges = graph.get(startState);
      edges.add(new Pair<T, A>(endState, weight));
      graph.put(startState, edges);
    } else {
      throw new IllegalArgumentException();
    }
  }

  /**
   * Adds a node from start state to end state and end state to start state with a weight of weight
   *
   * @param state1 One of the states for the edges
   * @param state2 The other state for the edges
   * @param weight The weight of the edges
   * @throws IllegalArgumentException when either state1 or state2 are not valid nodes in the graph
   */
  public void addBoth(T state1, T state2, A weight) {
    if (graph.containsKey(state1) && graph.containsKey(state2)) {
      add(state1, state2, weight);
      add(state2, state1, weight);
    } else {
      throw new IllegalArgumentException();
    }
  }

  /**
   * Adds a new node to the graph
   *
   * @param node Node to be added to the graph
   */
  public void addNode(T node) {
    graph.put(node, new ArrayList<Pair<T, A>>());
  }

  /**
   * Returns edges connected to the given node
   *
   * @param node Node to get edges from
   * @return A list of edges from the provided node
   */
  public List<Pair<T, A>> getAdj(T node) {
    return graph.get(node);
  }
}
