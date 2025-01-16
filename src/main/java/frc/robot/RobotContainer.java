// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.utils.StateRequest;
import frc.robot.utils.SubsystemRegistry;

public class RobotContainer {

  // Add a subsystem field for each subsystem

  public RobotContainer() {
    StateRequest.init(SubsystemRegistry.getInstance(Superstructure.class));

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("Autonomous Command");
  }
}
