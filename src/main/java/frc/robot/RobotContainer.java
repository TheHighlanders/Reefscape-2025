// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);

  Elevator elevator = new Elevator();
  Swerve drive = new Swerve(elevator::getElevatorPosition);
  CoralScorer coralScorer;
  Autos autos;
  Climber climber;

  AutoChooser chooser;

  public RobotContainer() {
    if (Constants.CoralEnabled) coralScorer = new CoralScorer();
    if (Constants.AutosEnabled) autos = new Autos(drive);
    if (Constants.ClimberEnabled) climber = new Climber();
    if (Constants.AutosEnabled) chooser = new AutoChooser();

    configureBindings();
    if (Constants.AutosEnabled) {
      configureAutonomous();
    }
    if (Constants.SwerveEnabled)
      drive.setDefaultCommand(
          drive.driveCMD(driver::getLeftX, driver::getLeftY, driver::getRightX));
  }

  private void configureBindings() {
    driver.start().onTrue(drive.resetGyro());

    driver.y().onTrue(elevator.setPosition(ElevatorState.L4_POSITION));
    driver.x().onTrue(elevator.setPosition(ElevatorState.L3_POSITION));
    driver.b().onTrue(elevator.setPosition(ElevatorState.L2_POSITION));
    driver.a().onTrue(elevator.setPosition(ElevatorState.CORAL_POSITION));
    driver
        .rightTrigger(0.5)
        .onTrue(elevator.setPosition(ElevatorState.L1_POSITION).alongWith(coralScorer.intakeCMD()));

    driver.leftTrigger(0.5).onTrue(coralScorer.depositCMD());
    driver.leftTrigger().whileTrue(drive.slowMode());

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
