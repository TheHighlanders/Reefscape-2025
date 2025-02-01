// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.StateRequest;
import java.util.HashMap;
import java.util.Map;

public class RobotContainer {
  private final Map<String, Subsystem> subsystems = new HashMap<>();

  CommandXboxController driver = new CommandXboxController(0);

  Swerve drive = new Swerve();
  EndEffector endEffector = new EndEffector();
  Autos autos = new Autos(drive);

  public RobotContainer() {
    subsystems.put("drive", drive);
    subsystems.put("endEffector", endEffector);
    // This needs to be the last subsystem added
    Superstructure superstructure = new Superstructure(subsystems);
    subsystems.put("superstructure", superstructure);
    StateRequest.init(superstructure);

    configureBindings();

    drive.setDefaultCommand(drive.driveCMD(driver::getLeftX, driver::getLeftY, driver::getRightX));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autos.testTraj();
  }
}
