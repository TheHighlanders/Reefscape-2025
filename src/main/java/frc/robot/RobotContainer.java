// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ArmState;
import frc.robot.subsystems.Superstructure.IntakeState;
import frc.robot.utils.StateRequest;

public class RobotContainer {
  Superstructure superstructure;

  Elevator elevator;

  public RobotContainer() {
    superstructure = new Superstructure(elevator);
    StateRequest.init(superstructure);

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    StateRequest.create(ArmState.L4_POSITION).with(IntakeState.STOPPED);
    return Commands.print("Autonomous Command");
  }
}
