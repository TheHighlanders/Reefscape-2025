// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Swerve;
import java.util.HashMap;
import java.util.Map;

public class RobotContainer {

  private final Map<String, Subsystem> subsystems = new HashMap<>();
  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);

  CoralScorer coralScorer = new CoralScorer();
  Climber climber = new Climber();
  Elevator elevator = new Elevator();
  Swerve drive = new Swerve(elevator::getElevatorPosition);
  Autos autos = new Autos(drive);
  Algae algae = new Algae();

  AutoChooser chooser;

  public RobotContainer() {
    chooser = new AutoChooser();

    subsystems.put("drive", drive);
    subsystems.put("CoralScorer", coralScorer);
    subsystems.put("climber", climber);
    subsystems.put("elevator", elevator);

    configureBindings();
    configureAutonomous();

    drive.setDefaultCommand(drive.driveCMD(driver::getLeftX, driver::getLeftY, driver::getRightX));
  }

  private void configureBindings() {
    // Controls Spreadsheet \/
    // https://docs.google.com/spreadsheets/d/1bb3pvQep2hsePMl8YiyaagtdjtIkL8Z2Oqf_t2ND-68/edit?gid=0#gid=0
    operator.a().onTrue(elevator.setPosition(ElevatorState.HOME));
    operator.x().onTrue(elevator.setPosition(ElevatorState.L2_POSITION));
    operator.y().onTrue(elevator.setPosition(ElevatorState.L3_POSITION));
    operator.b().onTrue(elevator.setPosition(ElevatorState.L4_POSITION));

    driver.start().whileTrue(elevator.zeroElevator());
    driver.povRight().onTrue(drive.resetGyro());

    driver.rightTrigger(0.5).whileTrue(coralScorer.depositCMD());
    driver.rightBumper().whileTrue(coralScorer.manualIntakeCMD());

    driver.leftTrigger().onTrue(drive.enableSlowMode());
    driver.leftTrigger().onFalse(drive.disableSlowMode());

    // operator.y().whileTrue(climber.createClimbOutCommand());
    // operator.a().whileTrue(climber.createClimbInCommand());

    operator.povUp().whileTrue(elevator.jogElevator(2));
    operator.povDown().whileTrue(elevator.jogElevator(-2));

    driver.back().onTrue(Commands.runOnce(drive::resetEncoders, drive));
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
