// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FullArmSubsystem.ArmState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
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
    PREL4,
    PRECLIMB,
    CLIMB
  }

  Graph graph;
  State currentState = State.HOME;
  State wantedState = State.HOME;

  FullArmSubsystem fullArm;
  IntakeSubsystem intake;
  ClimbSubsystem climb;
  Drive drive;

  /** Creates a new Superstructure. */
  public Superstructure(
      Drive drive, FullArmSubsystem fullArm, IntakeSubsystem intake, ClimbSubsystem climb) {
    this.drive = drive;
    this.fullArm = fullArm;
    this.intake = intake;
    this.climb = climb;

    initGraph();
  }

  @Override
  public void periodic() {
    if (areAllSubsystemsAtCurrentState() && currentState.compareTo(wantedState) != 0)
      currentState = tryState();
    applyStates();
  }

  public void setWantedSuperState(State wantedState) {
    this.wantedState = wantedState;
  }

  private void applyStates() {
    // TO-DO: 2910-style methods
    switch (currentState) {
      case SINTAKE:
        stationIntake();
        break;
      case CGINTAKE:
        coralGroundIntake();
        break;
      case AGINTAKE:
        algaeGroundIntake();
        break;
      case LALGAE:
        lowAlgae();
        break;
      case HALGAE:
        highAlgae();
        break;
      case PROCESSOR:
        processor();
        break;
      case BARGE:
        barge();
        break;
      case L1:
        l1();
        break;
      case L2:
        l2();
        break;
      case L3:
        l3();
        break;
      case L4:
        l4();
        break;
      case PRECLIMB:
        preClimb();
        break;
      case CLIMB:
        climb();
        break;
      case HOME:
        home();
        break;
      default:
        break;
    }
  }

  private void home() {
    fullArm.setGoalMethod(ArmState.HOME, false);
  }

  private void stationIntake() {
    fullArm.setGoalMethod(ArmState.STATION_INTAKE, false);
    intake.tryState(IntakeState.INTAKING_CORAL);
  }

  private void coralGroundIntake() {
    fullArm.setGoalMethod(ArmState.GROUND_CORAL_INTAKE, false);
    intake.tryState(IntakeState.INTAKING_CORAL);
  }

  private void algaeGroundIntake() {
    fullArm.setGoalMethod(ArmState.GROUND_ALGAE_INTAKE, false);
    intake.tryState(IntakeState.INTAKING_ALGAE);
    ;
  }

  private void lowAlgae() {
    fullArm.setGoalMethod(ArmState.LOWALGAE, false);
    intake.tryState(IntakeState.DEALGAEFYING);
  }

  private void highAlgae() {
    fullArm.setGoalMethod(ArmState.HIGHALGAE, false);
    intake.tryState(IntakeState.DEALGAEFYING);
  }

  private void processor() {
    fullArm.setGoalMethod(ArmState.PROCESSOR, false);
  }

  private void barge() {
    fullArm.setGoalMethod(ArmState.NET, false);
  }

  private void l1() {
    fullArm.setGoalMethod(ArmState.L1, false);
  }

  private void l2() {
    fullArm.setGoalMethod(ArmState.L2, false);
  }

  private void l3() {
    fullArm.setGoalMethod(ArmState.L3, false);
  }

  private void l4() {
    fullArm.setGoalMethod(ArmState.L4, false);
  }

  private void preClimb() {
    fullArm.setGoalMethod(ArmState.PRECLIMB, false);
    climb.setSpeed(1.0);
  }

  private void climb() {
    fullArm.setGoalMethod(ArmState.CLIMB, false);
  }

  private boolean areAllSubsystemsAtCurrentState() {
    // TO-DO: Make doesMatchState() for each subsystem
    return fullArm.isArmAtDesiredState()
        && intake.isIntakeAtDesiredState()
        && climb.isClimbAtDesiredState();
  }

  public void initGraph() {
    graph = new Graph();
    graph.addBoth(State.HOME, State.L1, 2);
    graph.addBoth(State.HOME, State.L2, 1);
    graph.addBoth(State.HOME, State.L3, 1);
    graph.addBoth(State.HOME, State.PREL4, 1);
    graph.addBoth(State.PREL4, State.L4, 1);
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
    double[] minCost = new double[State.values().length];
    Arrays.fill(minCost, Double.MAX_VALUE);
    boolean[] isProcessed = new boolean[State.values().length];
    minCost[currentState.ordinal()] = 0;
    PriorityQueue<StateCostPair> queue =
        new PriorityQueue<>(
            new Comparator<StateCostPair>() {
              @Override
              public int compare(StateCostPair o1, StateCostPair o2) {
                return Double.compare(o1.cost, o2.cost);
              }
            });
    State[] previousState = new State[State.values().length];
    queue.offer(new StateCostPair(currentState, 0));
    while (!queue.isEmpty()) {
      StateCostPair currentPair = queue.poll();
      State current = currentPair.state;
      if (isProcessed[current.ordinal()]) continue;
      if (current.compareTo(wantedState) == 0) break;
      isProcessed[current.ordinal()] = true;
      for (StateCostPair neighborPair : graph.getAdj(current)) {
        State neighbor = neighborPair.state;
        double costToNeighbor = neighborPair.cost;
        if (minCost[current.ordinal()] + costToNeighbor < minCost[neighbor.ordinal()]) {
          minCost[neighbor.ordinal()] = minCost[current.ordinal()] + costToNeighbor;
          previousState[neighbor.ordinal()] = current;
        }
        queue.offer(new StateCostPair(neighbor, minCost[neighbor.ordinal()]));
      }
    }
    State last = wantedState;
    while (previousState[last.ordinal()].compareTo(currentState) != 0) {
      last = previousState[last.ordinal()];
    }
    return last;
  }
}

class Graph {
  HashMap<Superstructure.State, List<StateCostPair>> graph;

  public Graph() {
    graph = new HashMap<>();
    for (Superstructure.State state : Superstructure.State.values()) {
      graph.put(state, new ArrayList<StateCostPair>());
    }
  }

  public void add(
      Superstructure.State startState, Superstructure.State endState, double transitionCost) {
    if (graph.containsKey(startState)) {
      List<StateCostPair> edges = graph.get(startState);
      edges.add(new StateCostPair(endState, transitionCost));
      graph.put(startState, edges);
    } else {
      throw new IllegalArgumentException();
    }
  }

  public void addBoth(
      Superstructure.State startState, Superstructure.State endState, double transitionCost) {
    if (graph.containsKey(startState) && graph.containsKey(endState)) {
      add(startState, endState, transitionCost);
      add(endState, startState, transitionCost);
    } else {
      throw new IllegalArgumentException();
    }
  }

  public List<StateCostPair> getAdj(Superstructure.State state) {
    return graph.get(state);
  }
}

class StateCostPair {
  public Superstructure.State state;
  public double cost;

  public StateCostPair(Superstructure.State state, double cost) {
    this.state = state;
    this.cost = cost;
  }
}
