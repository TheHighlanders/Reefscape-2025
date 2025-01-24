// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Superstructure;
import frc.robot.utils.StateRequest;

import java.util.HashMap;
import java.util.Map;

public class RobotContainer {
  private final Map<String, Subsystem> subsystems = new HashMap<String, Subsystem>();
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);
  private final Climber climber = new Climber(controller::getRightY, controller::getRightX); 

  public RobotContainer() {
    subsystems.put("climber", climber);

    // This needs to be the last subsystem added
    Superstructure superstructure = new Superstructure(subsystems);
    subsystems.put("superstructure", superstructure);
    StateRequest.init(superstructure);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    StateRequest.create(Superstructure.SuperstructureState.L1);
    return Commands.print("Autonomous Command");
  }
}
