// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class Autos {
  private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("midStart-L1ID21");
  Swerve drive;
  AutoFactory autoFactory;
  
  private final Timer timer = new Timer();

  public Autos(Swerve drive) {
    this.drive = drive;

    autoFactory =
        new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::resetOdometry, // A function that resets the current robot pose to the provided
            // Pose2d
            drive::followTraj, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            drive // The drive subsystem
            );
  }

  public Command testTraj() {
    return autoFactory.trajectoryCmd("Test");
  }

  public AutoRoutine testTrajRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("test");
    AutoTrajectory test = routine.trajectory("Test");

    routine.active().onTrue(test.cmd());

    return routine;
  }


}
