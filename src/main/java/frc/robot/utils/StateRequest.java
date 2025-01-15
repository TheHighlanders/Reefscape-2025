package frc.robot.utils;

import frc.robot.subsystems.Superstructure.IntakeState;
import frc.robot.subsystems.Superstructure.ArmState;
import frc.robot.subsystems.Superstructure.ElevatorState;

// THIS IS A PLACEHOLDER FOR THE STATE REQUEST CLASS, 
// THIS IS NOT THE FINAL CLASS AND WILL BE REPLACED WITH ACTUAL STATES

public class StateRequest {
    private IntakeState intakeState = null;
    private ArmState armState = null;
    private ElevatorState elevatorState = null;

    public static StateRequest create(IntakeState state) {
        StateRequest request = new StateRequest();
        request.intakeState = state;
        return request;
    }

    public static StateRequest create(ArmState state) {
        StateRequest request = new StateRequest();
        request.armState = state;
        return request;
    }

    public static StateRequest create(ElevatorState state) {
        StateRequest request = new StateRequest();
        request.elevatorState = state;
        return request;
    }

    public StateRequest with(IntakeState state) {
        this.intakeState = state;
        return this;
    }

    public StateRequest with(ArmState state) {
        this.armState = state;
        return this;
    }

    public StateRequest with(ElevatorState state) {
        this.elevatorState = state;
        return this;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public ArmState getArmState() {
        return armState;
    }

    public ElevatorState getElevatorState() {
        return elevatorState;
    }
}
