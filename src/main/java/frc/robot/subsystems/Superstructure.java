// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  public Superstructure() {
  }

  public enum Goal {
    STOW,
    L1,
    L2,
    L3,
    L4,
    CORAL_STATION,
    INIT_CLIMB,
    EXEC_CLIMB
  }

  private class goalState {
    Goal desired = Goal.STOW;
    Goal current = Goal.STOW;
    Goal previous = Goal.STOW;
  }

  private goalState goal;
  private Timer goalTimer = new Timer();

  @Override
  public void periodic() {
    
  }

  private void setGoal(Goal newGoal) {
    if (goal.desired == newGoal)
      return;
    goal.desired = newGoal;
  }

  public Command setGoalCommand(Goal newGoal) {
    return startEnd(() -> setGoal(newGoal), () -> setGoal(Goal.STOW))
        .withName("Superstructure " + goal);
  }
}
