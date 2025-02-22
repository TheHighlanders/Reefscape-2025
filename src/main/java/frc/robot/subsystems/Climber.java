// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

final class ClimberConstants {
  static final int climberCurrentLimit = 40;

  static final int climbMotorID = 42;

  // Rotations on input shaft to output shaft including gearbox conversion
  static final double climberPCF = 360.0 / 337.5;

  static final double climberSoftLimit = 120;

  static final double climberHoldVoltage = 0;

  static final double timeToZeroClimber = 1; // Seconds
}

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private Timer holdTimer = new Timer();
  private SparkMax climbMotor = new SparkMax(ClimberConstants.climbMotorID, MotorType.kBrushless);

  double climberHoldVoltage;
  double climberPreviousPosition;

  public Climber() {
    SparkMaxConfig climberConfig = createClimberConfig();
    climbMotor.configure(
        climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climbMotor.getEncoder().setPosition(0);
    climberPreviousPosition = 0;
  }

  private SparkMaxConfig createClimberConfig() {
    SparkMaxConfig climberConfig = new SparkMaxConfig();
    climberConfig.encoder.positionConversionFactor(
        ClimberConstants.climberPCF); // Rot to deg conversion

    climberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    climberConfig.smartCurrentLimit(ClimberConstants.climberCurrentLimit).idleMode(IdleMode.kBrake);
    // climberConfig
    // .softLimit
    // .forwardSoftLimit(0)
    // .forwardSoftLimitEnabled(true)
    // .reverseSoftLimit(-ClimberConstants.climberSoftLimit)
    // .reverseSoftLimitEnabled(true);
    climberHoldVoltage = ClimberConstants.climberHoldVoltage;
    SmartDashboard.putNumber("Climber/ClimberHoldVoltage", climberHoldVoltage);

    return climberConfig;
  }

  public void periodic() {
    SmartDashboard.putNumber("Climber/ClimberPosition", climbMotor.getEncoder().getPosition());
    climberHoldVoltage = SmartDashboard.getNumber("Climber/ClimberHoldVoltage", ClimberConstants.climberHoldVoltage);
    SmartDashboard.putNumber("Climber/ClimberHoldVoltage", climberHoldVoltage);
  }

  public void findClimberZeroTick() {
    climbMotor.set(0.05);
    double currentPosition = climbMotor.getEncoder().getPosition();

    if (MathUtil.isNear(climberPreviousPosition, currentPosition, 0.1)) {
      if (!holdTimer.isRunning()) {
        holdTimer.start();
      }
    } else {
      holdTimer.reset();
    }
  }

  public void handleAtZeroPosition() {
    climbMotor.getEncoder().setPosition(0);
    climbMotor.set(0);
  }

  public Command createClimbOutCommand() {
    // TODO: make sure 1 is correct direction
    return Commands.startEnd(
        () -> climbMotor.set(-0.5),
        // Stop the climber at the end of the command
        () -> climbMotor.set(0.0),
        this);
  }

  public Command findZeroPosition() {
    return Commands.run(this::findClimberZeroTick, this)
        .until(() -> holdTimer.hasElapsed(ClimberConstants.timeToZeroClimber))
        .finallyDo(this::handleAtZeroPosition);
  }

  public Command createClimbInCommand() {
    return Commands.startEnd(() -> climbMotor.set(1), () -> climbMotor.set(0.0), this);
  }

  public Command holdClimbPosition() {
    return Commands.runOnce(() -> climbMotor.setVoltage(climberHoldVoltage), this);
  }
}
