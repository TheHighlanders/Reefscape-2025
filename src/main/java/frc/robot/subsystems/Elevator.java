// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public enum ElevatorState {
    FAST,
    SLOW
  }

  public ElevatorState state = ElevatorState.FAST;

  /** Creates a new Elevator. */
  public Elevator() {

  }

  @Override
  public void periodic() {
    switch (state) {
      case SLOW:
        Commands.print("Slow").schedule();
        break;
      default:
        break;
    }
  }
}
