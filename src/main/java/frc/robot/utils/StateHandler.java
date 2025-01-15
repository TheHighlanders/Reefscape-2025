package frc.robot.utils;

import java.util.function.Consumer;

public class StateHandler<T> {
    private final SubsystemGoal<T> goal;
    private final Consumer<T> handleState;

    public StateHandler(T initialState, Consumer<T> handleState) {
        this.goal = new SubsystemGoal<>(initialState);
        this.handleState = handleState;
    }

    public void handle() {
        if (!goal.isAtGoal()) {
            handleState.accept(goal.desired);
        }
    }

    public SubsystemGoal<T> getGoal() {
        return goal;
    }
}
