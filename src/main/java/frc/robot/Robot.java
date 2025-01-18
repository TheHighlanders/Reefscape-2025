// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.subsystems.Superstructure.IntakeState;
// import frc.robot.utils.StateRequest;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private SparkMax elevatorMotor;
  private SparkMaxConfig elevatorMotorConfig;
  private SparkLimitSwitch bottomLimitSwitch;
  private RelativeEncoder elevatorEncoder;

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();

    elevatorMotor = new SparkMax(1, MotorType.kBrushless);
    bottomLimitSwitch = elevatorMotor.getReverseLimitSwitch();
    elevatorEncoder = elevatorMotor.getEncoder();

    elevatorMotorConfig = new SparkMaxConfig();

    elevatorMotorConfig.idleMode(IdleMode.kBrake);

    elevatorMotorConfig.limitSwitch
       .forwardLimitSwitchType(Type.kNormallyOpen)
       .forwardLimitSwitchEnabled(true)
       .reverseLimitSwitchType(Type.kNormallyOpen)
       .bottomLimitSwitchEnabled(true);


    elevatorMotorConfig.softLimit
    .forwardSoftLimit(50)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimit(-50)
    .reverseSoftLimitEnabled(true);

    elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Reset the position to 0 to start within the range of the soft limits
    elevatorEncoder.setPosition(0);

    // Initialize dashboard values
    SmartDashboard.setDefaultBoolean("Direction", true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

        // Display data from SPARK onto the dashboard
        SmartDashboard.putBoolean("Reverse Limit Reached", bottomLimitSwitch.isPressed());
        SmartDashboard.putNumber("Applied Output", elevatorMotor.getAppliedOutput());
        SmartDashboard.putNumber("Position", elevatorEncoder.getPosition());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

        // Set the motor setpoint based on the direction from the dashboard
        if (SmartDashboard.getBoolean("Direction", true)) {
          elevatorMotor.set(0.2);
        } else {
          elevatorMotor.set(-0.2);
        }
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
