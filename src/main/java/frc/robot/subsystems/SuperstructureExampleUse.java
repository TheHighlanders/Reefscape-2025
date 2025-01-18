package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.StateRequest;

public class SuperstructureExampleUse extends SubsystemBase {
  public enum FirstEnumState {
    ONE,
    TWO,
    THREE,
    FOUR,
    FIVE
  }

  public enum SecondEnumState {
    A,
    B,
    C
  }

  public FirstEnumState state = FirstEnumState.ONE;
  public SecondEnumState testState = SecondEnumState.A;

  public SuperstructureExampleUse() {}

  public void runTests() {
    StateRequest.addOneWayExclusion(FirstEnumState.ONE, FirstEnumState.THREE);

    StateRequest.create(FirstEnumState.ONE);
    StateRequest.create(FirstEnumState.THREE);
    Commands.print(
            "Forward Exclusion Test: "
                + (StateRequest.getCurrentState(FirstEnumState.class) == FirstEnumState.ONE
                    ? "PASSED"
                    : "FAILED"))
        .schedule();

    StateRequest.create(FirstEnumState.THREE);
    StateRequest.create(FirstEnumState.ONE);
    Commands.print(
            "Reverse Direction Test: "
                + (StateRequest.getCurrentState(FirstEnumState.class) == FirstEnumState.ONE
                    ? "PASSED"
                    : "FAILED"))
        .schedule();

    var stateChanged = StateRequest.createStateChangeSupplier(FirstEnumState.class);
    StateRequest.create(FirstEnumState.TWO);
    Commands.print("State Change Test: " + (stateChanged.getAsBoolean() ? "PASSED" : "FAILED"))
        .schedule();

    var stateChanged2 = StateRequest.createStateChangeSupplier(FirstEnumState.class);
    StateRequest.create(FirstEnumState.TWO);
    Commands.print("State Change Test: " + (stateChanged2.getAsBoolean() ? "FAILED" : "PASSED"))
        .schedule();

    // Test cross-enum exclusions
    StateRequest.addOneWayExclusion(FirstEnumState.ONE, SecondEnumState.B);
    StateRequest.create(FirstEnumState.ONE);
    StateRequest.create(SecondEnumState.B);

    Commands.print(
            "Cross-Enum Exclusion Test: "
                + (StateRequest.getCurrentState(SecondEnumState.class) == SecondEnumState.A
                    ? "PASSED"
                    : "FAILED"))
        .schedule();
  }

  @Override
  public void periodic() {}
}
