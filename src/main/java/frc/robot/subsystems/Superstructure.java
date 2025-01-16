// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.StateHandler;
import frc.robot.utils.StateManagedSubsystem;
import frc.robot.utils.SubsystemRegistry;

public class Superstructure extends SubsystemBase {
  // Exampes of states for the superstructure. Not actual states.

  public final Map<Class<? extends Enum<?>>, StateHandler<?>> handlers = new HashMap<>();

  // Take in the subsytem as an argument and initialize it to its field
  // Add the subsystem to the handlers map with its Enum class as the key
  public Superstructure() {
    // Get all state-managed subsystems from registry and add their handlers
    SubsystemRegistry.getAllStateSubsystems().stream()
        .filter(subsystem -> subsystem instanceof StateManagedSubsystem)
        .map(subsystem -> (StateManagedSubsystem) subsystem)
        .forEach(subsystem -> handlers.put(subsystem.getStateType(), subsystem.getStateHandler()));
  }

  @Override
  public void periodic() {
    handlers.values().forEach(StateHandler::handle);
  }

}
