// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.utils.StateManagedSubsystemBase;

public class Elevator extends StateManagedSubsystemBase<Elevator.ElevatorState> {
  public enum ElevatorState {
    FAST,
    SLOW
  }

  /** Creates a new Elevator. */
  public Elevator() {
  }

  @Override
  public void periodic() {

  }
}
