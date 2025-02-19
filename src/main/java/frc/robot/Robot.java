// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  // Loads a swerve trajectory, alternatively use DifferentialSample if the robot is tank drive
  private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("midStart -> L1ID21");
  private final Drive driveSubsystem = new Drive();
  private final Timer timer = new Timer ();
  
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private double loops = 0;

  public Robot() {
    m_robotContainer = new RobotContainer();
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if (loops % 50 == 0) {
      m_robotContainer.drive.updateControlConstants();

      loops = 0;
    } else if (loops % 25 == 0) {
      m_robotContainer.drive.resetEncoders();
    }
    loops++;
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
       if (trajectory.isPresent()) {
            // Get the initial pose of the trajectory
            Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());

            if (initialPose.isPresent()) {
                // Reset odometry to the start of the trajectory
                driveSubsystem.resetOdometry(initialPose.get());
            }
        }

        // Reset and start the timer when the autonomous period begins
        timer.restart();
  }

  @Override
  public void autonomousPeriodic() {
    if (trajectory.isPresent()) {
      // Sample the trajectory at the current time into the autonomous period
      Optional<SwerveSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());

      if (sample.isPresent()) {
          driveSubsystem.followTrajectory(sample);
      }
  }
  private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  } 

  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
