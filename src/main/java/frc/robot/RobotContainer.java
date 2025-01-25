// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructureExampleUse;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.StateRequest;

public class RobotContainer {
  private final Set<Subsystem> subsystems = new HashSet<>();
  SuperstructureExampleUse example;

  CommandXboxController driver = new CommandXboxController(0);

  Swerve drive = new Swerve();
  Autos autos = new Autos(drive);

  public RobotContainer() {
    example = new SuperstructureExampleUse();
    subsystems.add(example);

    // This needs to be the last subsystem added
    Superstructure superstructure = new Superstructure(subsystems);
    subsystems.add(superstructure);
    StateRequest.init(superstructure);

    configureBindings();

    drive.setDefaultCommand(drive.driveCMD(driver::getLeftX, driver::getLeftY, driver::getRightX));
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return autos.testTraj();
  }
}
