// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructureExampleUse;
import frc.robot.utils.StateRequest;
import java.util.HashSet;
import java.util.Set;

public class RobotContainer {
  private final Set<Subsystem> subsystems = new HashSet<>();
  SuperstructureExampleUse example;

  public RobotContainer() {
    example = new SuperstructureExampleUse();
    subsystems.add(example);

    // This needs to be the last subsystem added
    Superstructure superstructure = new Superstructure(subsystems);
    subsystems.add(superstructure);
    StateRequest.init(superstructure);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    example.runTests();
    return Commands.print("Autonomous Command");
  }
}
