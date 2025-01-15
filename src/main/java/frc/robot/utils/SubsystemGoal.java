package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;

public class SubsystemGoal<T> {
    T desired;
    T current;
    T previous;
    Timer transitionTimer = new Timer();
    boolean atGoal = true;

    public SubsystemGoal(T initialState) {
        desired = initialState;
        current = initialState;
        previous = initialState;
        transitionTimer.start();
    }

    public void updateState(T newDesired) {
        Commands.print("Changing desired state to: " + newDesired).schedule();
        if (desired != newDesired) {
            previous = current;
            desired = newDesired;
            atGoal = false;
            transitionTimer.reset();
        }
    }

    public void achieveGoal() {
        if (current != desired) {
            previous = current;
            current = desired;
            atGoal = true;
            transitionTimer.stop();
        }
    }

    public double getTransitionTime() {
        return transitionTimer.get();
    }

    public boolean isAtGoal() {
        return atGoal;
    }
}