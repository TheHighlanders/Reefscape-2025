// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@Logged
public class Robot extends TimedRobot {

  private final LEDS leds;
  
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;



  private double loops = 0;

  public Robot() {
    m_robotContainer = new RobotContainer();
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    DataLogManager.start();

    /*
     * https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-
     * with-annotations.html
     */
    Epilogue.configure(
        config -> {
          // Log only to disk, instead of the default NetworkTables logging
          // Note that this means data cannot be analyzed in realtime by a dashboard
          config.backend = new FileBackend(DataLogManager.getLog());

          if (isSimulation()) {
            // If running in simulation, then we'd want to re-throw any errors that
            // occur so we can debug and fix them!
            config.errorHandler = ErrorHandler.crashOnError();
          }

          // Change the root data path
          config.root = "Telemetry";

          // Only log critical information instead of the default DEBUG level.
          // This can be helpful in a pinch to reduce network bandwidth or log file size
          // while still logging important information.
          config.minimumImportance = Logged.Importance.DEBUG;
        });
    DriverStation.startDataLog(DataLogManager.getLog());
    Epilogue.bind(this);

    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public void disabledInit() {

    if (Constants.devMode) {
      m_robotContainer.elevator.sendTuningConstants();
    }
  }

  @Override
  public void disabledPeriodic() {
    if (loops % 50 == 0) {
      loops = 0;
    } else if (loops % 25 == 0) {
      m_robotContainer.drive.attemptZeroingAbsolute();
    }
    if (loops % 50 == 37) {
      if (Constants.devMode) {
        m_robotContainer.drive.updateControlConstants();
        m_robotContainer.drive.updateDashboardGUI();
        m_robotContainer.drive.updateTrajectoryPID();
        m_robotContainer.elevator.updateTuningConstants();
      }
    }
    loops++;

    leds.
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    WebServer.stop(5800);
  }

  @Override
  public void autonomousPeriodic() {}

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
