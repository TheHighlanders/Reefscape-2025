package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;

public class SubsystemGoal<T> {
    T desired;
    T current;
    T previous;
    Timer transitionTimer = new Timer();
    boolean atGoal = true;
    private final List<Runnable> stateChangeCallbacks = new ArrayList<>();

    public SubsystemGoal(T initialState) {
        desired = initialState;
        current = initialState;
        previous = initialState;
        transitionTimer.start();
    }

    public void updateState(T newDesired) {
        if (desired != newDesired) {
            previous = current;
            desired = newDesired;
            atGoal = false;
            transitionTimer.reset();
            stateChangeCallbacks.forEach(Runnable::run);
        }
    }

    public void achieveGoal() {
        if (current != desired) {
            previous = current;
            current = desired;
            atGoal = true;
            transitionTimer.stop();
            stateChangeCallbacks.forEach(Runnable::run);
        }
    }

    public void addStateChangeCallback(Runnable callback) {
        stateChangeCallbacks.add(callback);
    }

    @Override
    public String toString() {
        return String.format(
                "SubsystemGoal{desired=%s, current=%s, previous=%s, transitionTime=%.2f, atGoal=%b}",
                desired,
                current,
                previous,
                transitionTimer.get(),
                atGoal);
    }

    public double getTransitionTime() {
        return transitionTimer.get();
    }

    public boolean isAtGoal() {
        return atGoal;
    }

    public T currentState() {
        return current;
    }

    public T desiredState() {
        return desired;
    }

    public T previousState() {
        return previous;
    }
}