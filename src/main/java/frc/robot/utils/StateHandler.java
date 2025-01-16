package frc.robot.utils;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;

public class StateHandler<T> {
    private final SubsystemGoal<T> goal;
    private final Consumer<T> handleState;
    private final Map<T, Set<T>> stateExclusions;

    public StateHandler(T initialState, Consumer<T> handleState) {
        this.goal = new SubsystemGoal<>(initialState);
        this.handleState = handleState;
        this.stateExclusions = new HashMap<>();
    }

    public StateHandler(T initialState, Consumer<T> handleState, Map<T, Set<T>> stateExclusions) {
        this.goal = new SubsystemGoal<>(initialState);
        this.handleState = handleState;
        this.stateExclusions = new HashMap<>(stateExclusions);
    }

    public void addOneWayExclusion(T fromState, T excludedState) {
        stateExclusions.computeIfAbsent(fromState, k -> new HashSet<>()).add(excludedState);
    }

    public void addTwoWayExclusion(T stateA, T stateB) {
        stateExclusions.computeIfAbsent(stateA, k -> new HashSet<>()).add(stateB);
        stateExclusions.computeIfAbsent(stateB, k -> new HashSet<>()).add(stateA);
    }

    public void handle() {
        T currentState = goal.current;
        Set<T> exclusions = stateExclusions.getOrDefault(currentState, new HashSet<>());
        
        if (!goal.isAtGoal() && !exclusions.contains(goal.desired)) {
            handleState.accept(goal.desired);
        }
    }

    public SubsystemGoal<T> getGoal() {
        return goal;
    }
}
