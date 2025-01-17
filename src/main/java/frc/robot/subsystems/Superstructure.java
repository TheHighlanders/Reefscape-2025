// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.StateHandler;
import frc.robot.utils.StateManagedSubsystem;

public class Superstructure extends SubsystemBase {
  public final Map<Class<? extends Enum<?>>, StateHandler<? extends Enum<?>>> handlers = new HashMap<>();

  @SuppressWarnings("unchecked")
  public Superstructure(Set<Subsystem> subsystems) {
    subsystems.stream()
        .filter(subsystem -> subsystem instanceof StateManagedSubsystem)
        .map(subsystem -> (StateManagedSubsystem) subsystem)
        .forEach(subsystem -> handlers.put(subsystem.getStateType(), subsystem.getStateHandler()));
  }

  @Override
  public void periodic() {}
}
