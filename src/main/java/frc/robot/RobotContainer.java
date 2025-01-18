// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure;
import frc.robot.utils.StateRequest;

public class RobotContainer {
  private final Set<Subsystem> subsystems = new HashSet<>();

  public RobotContainer() {
    Elevator elevator = new Elevator();
    subsystems.add(elevator);

    // This needs to be the last subsystem added
    Superstructure superstructure = new Superstructure(subsystems);
    subsystems.add(superstructure);
    StateRequest.init(superstructure);

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    StateRequest.addTwoWayExclusion(Elevator.ElevatorState.SLOW, Elevator.ElevatorState.FAST);
    StateRequest.create(Elevator.ElevatorState.SLOW);
    return Commands.print("Autonomous Command");
  }
}
