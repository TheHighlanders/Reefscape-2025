// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.StateRequest;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;

  // Exampes of states for the superstructure. Not actual states.

  public enum IntakeState {
    STOPPED,
    INTAKING,
    EJECTING,
    HOLDING
  }

  public enum ArmState {
    STOWED,
    INTAKE_POSITION,
    L1_POSITION,
    L2_POSITION,
    L3_POSITION,
    L4_POSITION,
    CLIMB_PREP,
    CLIMBING,
    CORAL_POSITION
  }

  public enum ElevatorState {
    HOME,
    CLIMBING
  }

  private class SubsystemGoal<T> {
    T desired;
    T current;
    T previous;
    Timer transitionTimer = new Timer();

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
        transitionTimer.reset();
      }
    }

    public double getTransitionTime() {
      return transitionTimer.get();
    }
  }

  private final SubsystemGoal<IntakeState> intakeGoal = new SubsystemGoal<>(IntakeState.STOPPED);
  private final SubsystemGoal<ArmState> armGoal = new SubsystemGoal<>(ArmState.STOWED);
  private final SubsystemGoal<ElevatorState> elevatorGoal = new SubsystemGoal<>(ElevatorState.HOME);

  public Superstructure(Elevator elevator) {
    // Normally would take in all subsystems but for example purposes only
    this.elevator = elevator;
  }

  @Override
  public void periodic() {
    handleIntakeState();
    handleArmState();
    handleElevatorState();
  }

  private void handleIntakeState() {
    switch (intakeGoal.desired) {
      case STOPPED:
        // Will call intake.stop()
        break;
      case INTAKING:
        // Will call intake.intake()
        break;
      case EJECTING:
        // Will call intake.eject()
        break;
      case HOLDING:
        // Will call intake.hold()
        break;
    }
  }

  private void handleArmState() {
    switch (armGoal.desired) {
      case STOWED:
        // Will call arm.setStowPosition()
        break;
      case INTAKE_POSITION:
        // Will call arm.setIntakePosition()
        break;
      case L1_POSITION:
        // Will call arm.setL1Position()
        break;
      case L2_POSITION:
        // Will call arm.setL2Position()
        break;
      case L3_POSITION:
        // Will call arm.setL3Position()
        break;
      case L4_POSITION:
        // Will call arm.setL4Position()
        break;
      case CLIMB_PREP:
        // Will call arm.setClimbPrep()
        break;
      case CLIMBING:
        // Will call arm.executeClimb()
        break;
      case CORAL_POSITION:
        // Will call arm.setCoralPosition()
        break;
    }
  }

  private void handleElevatorState() {
    switch (elevatorGoal.desired) {
      case HOME:
        // Will call elevator.setHomePosition()
        break;
      case CLIMBING:
        // Will call elevator.executeClimb()
        break;
    }
  }

  private void setState(StateRequest request) {
    if (request.getIntakeState() != null)
      intakeGoal.updateState(request.getIntakeState());
    if (request.getArmState() != null)
      armGoal.updateState(request.getArmState());
    if (request.getElevatorState() != null)
      elevatorGoal.updateState(request.getElevatorState());
  }

  // Example of a command that will set the superstructure to a specific state

  public Command getScoreL1Command() {
    return startEnd(
        () -> setState(StateRequest.create(ArmState.L1_POSITION)
            .with(ElevatorState.HOME)
            .with(IntakeState.EJECTING)),
        () -> setState(StateRequest.create(ArmState.STOWED))).withName("Score L1");
  }
}
