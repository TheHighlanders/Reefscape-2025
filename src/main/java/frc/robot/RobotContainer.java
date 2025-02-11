// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import java.util.HashMap;
import java.util.Map;

public class RobotContainer {

  private final Map<String, Subsystem> subsystems = new HashMap<>();
  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);

  Elevator elevator = new Elevator();
  Swerve drive = new Swerve(elevator::getElevatorPosition);
  CoralScorer coralScorer;
  Autos autos;
  Climber climber;

  AutoChooser chooser;

  public RobotContainer() {
    if (!Constants.onlyConstructSwerve) {
      coralScorer = new CoralScorer();
      autos = new Autos(drive);
      climber = new Climber();
      chooser = new AutoChooser();

      subsystems.put("endEffector", coralScorer);
      subsystems.put("climber", climber);
      subsystems.put("elevator", elevator);
    }

    subsystems.put("drive", drive);

    configureBindings();
    if (!Constants.onlyConstructSwerve) {
      configureAutonomous();
    }

    drive.setDefaultCommand(drive.driveCMD(driver::getLeftX, driver::getLeftY, driver::getRightX));
  }

  private void configureBindings() {
    driver.x().onTrue(drive.resetGyro());

    if (!Constants.onlyConstructSwerve) {
      driver.a().whileTrue(climber.createClimbInCommand());
      driver.b().whileTrue(climber.createClimbOutCommand());
    }
  }

  private void configureAutonomous() {
    chooser.addRoutine("Test Routine", autos::testTrajRoutine);
    chooser.addCmd("SYSID", drive::sysId);

    SmartDashboard.putData("AutoChooser", chooser);
  }

  public Command getAutonomousCommand() {
    return chooser.selectedCommand();
  }
}
