// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.StateHandler;
import frc.robot.utils.StateRequest;
import frc.robot.utils.SubsystemGoal;

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
    L1_POSITION,
    L2_POSITION,
    L3_POSITION,
    L4_POSITION,
    CORAL_POSITION
  }

  public final Map<Class<? extends Enum<?>>, StateHandler<?>> handlers = new HashMap<>();

  public Superstructure(Elevator elevator) {
    // Normally would take in all subsystems but they will throw errors if they dont
    // actually exist
    this.elevator = elevator;

    handlers.put(IntakeState.class, new StateHandler<>(IntakeState.STOPPED, this::handleIntakeState));
    handlers.put(ArmState.class, new StateHandler<>(ArmState.STOWED, this::handleArmState));
    handlers.put(ElevatorState.class, new StateHandler<>(ElevatorState.HOME, this::handleElevatorState));
  }

  @Override
  public void periodic() {
    handlers.values().forEach(StateHandler::handle);
  }

  private void handleIntakeState(IntakeState state) {
    switch (state) {
      case STOPPED:
        Commands.print("STOPPED").schedule();
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

  private void handleArmState(ArmState state) {
    switch (state) {
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
        Commands.print("L4_POSITION").schedule();
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

  private void handleElevatorState(ElevatorState state) {
    switch (state) {
      case HOME:
        // Will call elevator.setHomePosition()
        break;
      case L1_POSITION:
        // Will call elevator.setL1Position()
        break;
      case L2_POSITION:
        // Will call elevator.setL2Position()
        break;
      case L3_POSITION:
        // Will call elevator.setL3Position()
        break;
      case L4_POSITION:
        // Will call elevator.setL4Position()
        break;
      case CORAL_POSITION:
        // Will call elevator.setCoralPosition()
        break;
    }
  }

  // Example commands that will set the superstructure to a specific state

  public Command getScoreL1Command() {
    return startEnd(
        () -> StateRequest.create(ArmState.L1_POSITION)
            .with(ElevatorState.HOME)
            .with(IntakeState.EJECTING),
        () -> StateRequest.create(ArmState.STOWED)).withName("Score L1");
  }

  public Command getIntakeCommand() {
    return startEnd(
        () -> StateRequest.create(IntakeState.INTAKING),
        () -> StateRequest.create(IntakeState.STOPPED)).withName("Intake");
  }
}
