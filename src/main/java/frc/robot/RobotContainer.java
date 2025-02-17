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
    // driver.start().onTrue(drive.resetGyro());

    driver.x().onTrue(elevator.setPosition(ElevatorState.L3_POSITION));
    driver.y().onTrue(elevator.setPosition(ElevatorState.L2_POSITION));
    driver.a().onTrue(elevator.setPosition(ElevatorState.HOME));
    driver.b().onTrue(elevator.setPosition(ElevatorState.L4_POSITION));
    driver.povDown().whileTrue(elevator.jogElevator(-0.3));
    driver.povUp().whileTrue(elevator.jogElevator(0.3));

    driver.start().whileTrue(elevator.zeroElevator());
    // driver
    //     .rightTrigger(0.5)
    //
    // .onTrue(elevator.setPosition(ElevatorState.L1_POSITION).alongWith(CoralScorer.intakeCMD()));

    driver.leftTrigger(0.5).whileTrue(coralScorer.depositCMD());
    driver.rightTrigger(0.5).whileTrue(coralScorer.intakeCMD());
    // driver.leftTrigger().whileTrue(drive.slowMode());

    operator.y().whileTrue(climber.createClimbOutCommand());
    operator.a().whileTrue(climber.createClimbInCommand());
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
