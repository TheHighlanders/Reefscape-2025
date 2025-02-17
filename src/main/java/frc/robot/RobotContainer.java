// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Swerve;
import java.util.HashMap;
import java.util.Map;

public class RobotContainer {

  private final Map<String, Subsystem> subsystems = new HashMap<>();
  CommandXboxController driver = new CommandXboxController(0);

  Swerve drive = new Swerve();
  EndEffector endEffector = new EndEffector();
  Autos autos = new Autos(drive);
  Climber climber = new Climber();
  Elevator elevator = new Elevator();
  Algae algae = new Algae();

  AutoChooser chooser;

  public RobotContainer() {
    chooser = new AutoChooser();

    subsystems.put("drive", drive);
    subsystems.put("endEffector", endEffector);
    subsystems.put("climber", climber);
    subsystems.put("elevator", elevator);

    configureBindings();
    configureAutonomous();

    drive.setDefaultCommand(drive.driveCMD(driver::getLeftX, driver::getLeftY, driver::getRightX));
  }

  private void configureBindings() {
    driver.x().onTrue(drive.resetGyro());
    driver.a().whileTrue(climber.createClimbInCommand());
    driver.b().whileTrue(climber.createClimbOutCommand());

    driver.rightBumper().whileTrue(algae.intakeAlgae());
    driver.leftBumper().whileTrue(algae.outputAlgae());
    driver.back().onTrue(algae.brushedIntake());
    driver.start().onTrue(algae.outputBrushed());
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
