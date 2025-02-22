// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class Autos {
  AutoFactory autoFactory;
  Swerve drive;
  Elevator elevator;
  CoralScorer coral;

  public Autos(Swerve drive, Elevator elevator, CoralScorer coral) {
    autoFactory =
        new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::resetOdometry, // A function that resets the current robot pose to the provided
            // Pose2d
            drive::followTraj, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            drive // The drive subsystem
            );

    this.drive = drive;
    this.elevator = elevator;
    this.coral = coral;
  }

  public Command testTraj() {
    return autoFactory.trajectoryCmd("Test");
  }

  public AutoRoutine testDriveTrajRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("testDrive");
    AutoTrajectory test = routine.trajectory("TestDrive");

    routine.active().onTrue(Commands.sequence(updateTrajectoryPIDCMD(), test.cmd()));

    return routine;
  }

  public AutoRoutine testRotateTrajRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("testDriveRotate");
    AutoTrajectory test = routine.trajectory("TestRotate");

    routine.active().onTrue(Commands.sequence(updateTrajectoryPIDCMD(), test.cmd()));

    return routine;
  }

  public AutoRoutine testDriveRotateTrajRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("testDriveRotate");
    AutoTrajectory test = routine.trajectory("testRotateAndDrive");

    routine.active().onTrue(Commands.sequence(updateTrajectoryPIDCMD(), test.cmd()));

    return routine;
  }

  public Command updateTrajectoryPIDCMD() {
    return Commands.runOnce(drive::updateTrajectoryPID);
  }

  public Command simpleDriveAuto() {
    return drive.driveForwardTimed(1, 4);
  }

  public Command simple1Piece() {
    return Commands.sequence(drive.driveForwardTimed(1.5, 1.5));
  }
}
