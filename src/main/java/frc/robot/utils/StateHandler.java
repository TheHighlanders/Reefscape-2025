package frc.robot.utils;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class StateHandler<T extends Enum<T>> {
    private final SubsystemGoal<T> goal;
    private final Map<T, Set<T>> stateExclusions;
    

    public StateHandler(T initialState) {
        this.goal = new SubsystemGoal<>(initialState);
        this.stateExclusions = new HashMap<>();
    }

    public StateHandler(T initialState, Map<T, Set<T>> stateExclusions) {
        this.goal = new SubsystemGoal<>(initialState);
        this.stateExclusions = new HashMap<>(stateExclusions);
    }

    public void addOneWayExclusion(T fromState, T excludedState) {
        stateExclusions.computeIfAbsent(fromState, k -> new HashSet<>()).add(excludedState);
    }

    public void addTwoWayExclusion(T stateA, T stateB) {
        stateExclusions.computeIfAbsent(stateA, k -> new HashSet<>()).add(stateB);
        stateExclusions.computeIfAbsent(stateB, k -> new HashSet<>()).add(stateA);
    }

    public void onStateChange(Runnable callback) {
        goal.addStateChangeCallback(callback);
    }

    public SubsystemGoal<T> getSubsystemStates() {
        return goal;
    }

    public T getCurrentState() {
        return goal.currentState();
    }

    public T getDesiredState() {
        return goal.desiredState();
    }

    public T getPreviousState() {
        return goal.previousState();
    }
}
