// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// PLACEHOLDER FOR SUPERSTRUCTURE EXAMPLE



public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public enum ElevatorState {
    HOME,
    L1_POSITION,
    L2_POSITION,
    L3_POSITION,
    L4_POSITION,
    CORAL_POSITION
  }
  private ElevatorState uppydowny = ElevatorState.HOME;

  public Elevator() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (uppydowny) {
      case HOME:
        break;
      case L1_POSITION:
        break;
      case L2_POSITION:
        break;
      case L3_POSITION:
        break;
        case L4_POSITION:
        break;
    }
  }
}
