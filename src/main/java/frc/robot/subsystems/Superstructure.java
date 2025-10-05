// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.*;

public class Superstructure extends SubsystemBase {

  public enum State {
    HOME,
    SINTAKE,
    CGINTAKE,
    AGINTAKE,
    LALGAE,
    HALGAE,
    PROCESSOR,
    BARGE,
    L1,
    L2,
    L3,
    L4,
    PRECLIMB,
    CLIMB
  }

  Graph graph;
  State currentState = State.HOME;
  State wantedState = State.HOME;

  /** Creates a new Superstructure. */
  public Superstructure(
      Drive drive, FullArmSubsystem fullArm, IntakeSubsystem intake, ClimbSubsystem climb) {
    initGraph();
  }

  @Override
  public void periodic() {
    if (doesMatchState() && currentState.compareTo(wantedState) != 0) currentState = tryState();
    applyStates();
  }

  public void initGraph() {
    graph = new Graph();
    graph.addBoth(State.HOME, State.L1, 2);
    graph.addBoth(State.HOME, State.L2, 1);
    graph.addBoth(State.HOME, State.L3, 1);
    graph.addBoth(State.HOME, State.L4, 1);
    graph.addBoth(State.HOME, State.LALGAE, 1);
    graph.addBoth(State.HOME, State.HALGAE, 2);
    graph.addBoth(State.HOME, State.SINTAKE, 1);
    graph.addBoth(State.HOME, State.AGINTAKE, 1);
    graph.addBoth(State.HOME, State.CGINTAKE, 1);
    graph.addBoth(State.HOME, State.PRECLIMB, 1);
    graph.addBoth(State.HOME, State.PROCESSOR, 1);
    graph.addBoth(State.HOME, State.BARGE, 1);
    graph.addBoth(State.L1, State.SINTAKE, 2);
    graph.addBoth(State.L1, State.CGINTAKE, 1);
    graph.addBoth(State.L2, State.LALGAE, 1);
    graph.addBoth(State.L3, State.LALGAE, 1);
    graph.addBoth(State.L3, State.HALGAE, 1);
    graph.addBoth(State.L4, State.HALGAE, 1);
    graph.add(State.PRECLIMB, State.CLIMB, 1);
    graph.addBoth(State.AGINTAKE, State.CGINTAKE, 1);
    graph.addBoth(State.AGINTAKE, State.PROCESSOR, 1);
    graph.addBoth(State.AGINTAKE, State.SINTAKE, 1);
    graph.addBoth(State.SINTAKE, State.CGINTAKE, 1);
  }

  public State tryState() {
    double[] min = new double[State.values().length];
    Arrays.fill(min, Double.MAX_VALUE);
    boolean[] processed = new boolean[State.values().length];
    min[currentState.ordinal()] = 0;
    PriorityQueue<Pair> q =
        new PriorityQueue<>(
            new Comparator<Pair>() {
              @Override
              public int compare(Pair o1, Pair o2) {
                return Double.compare(o1.weight, o2.weight);
              }
            });
    State[] pr = new State[State.values().length];
    q.offer(new Pair(currentState, 0));
    while (!q.isEmpty()) {
      Pair p = q.poll();
      if (processed[p.state.ordinal()]) continue;
      if (p.state.compareTo(wantedState) == 0) break;
      processed[p.state.ordinal()] = true;
      for (Pair c : graph.getAdj(p.state)) {
        if (min[p.state.ordinal()] + c.weight < min[c.state.ordinal()]) {
          min[c.state.ordinal()] = min[p.state.ordinal()] + c.weight;
          pr[c.state.ordinal()] = p.state;
        }
        q.offer(new Pair(c.state, min[c.state.ordinal()]));
      }
    }
    State last = wantedState;
    while (pr[last.ordinal()].compareTo(currentState) != 0) {
      last = pr[last.ordinal()];
    }
    return last;
  }

  public void setWantedSuperState(State wantedState) {
    this.wantedState = wantedState;
  }

  private void applyStates() {
    // TO-DO: 2910-style methods
  }

  private boolean doesMatchState() {
    // TO-DO: Make doesMatchState() for each subsystem
    return true;
  }
}

class Graph {
  HashMap<Superstructure.State, List<Pair>> graph;

  public Graph() {
    graph = new HashMap<>();
    for (Superstructure.State s : Superstructure.State.values()) {
      graph.put(s, new ArrayList<Pair>());
    }
  }

  public void add(Superstructure.State i, Superstructure.State f, double w) {
    if (graph.containsKey(i)) {
      List<Pair> edges = graph.get(i);
      edges.add(new Pair(f, w));
      graph.put(i, edges);
    } else {
      throw new IllegalArgumentException();
    }
  }

  public void addBoth(Superstructure.State i, Superstructure.State f, double w) {
    if (graph.containsKey(i) && graph.containsKey(f)) {
      add(i, f, w);
      add(f, i, w);
    } else {
      throw new IllegalArgumentException();
    }
  }

  public List<Pair> getAdj(Superstructure.State s) {
    return graph.get(s);
  }
}

class Pair {
  public Superstructure.State state;
  public double weight;

  public Pair(Superstructure.State state, double weight) {
    this.state = state;
    this.weight = weight;
  }
}
