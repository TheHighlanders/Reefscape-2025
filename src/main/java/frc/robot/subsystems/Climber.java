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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

final class ClimberConstants {
  static final int climberCurrentLimit = 20;

  static final int climbMotorID = 42;

  // Rotations on input shaft to output shaft including gearbox conversion
  static final double climberPCF = 360.0 / 337.5;

  static final double climberSoftLimit = 80;
}

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private SparkMax climbMotor = new SparkMax(ClimberConstants.climbMotorID, MotorType.kBrushless);

  public Climber() {
    SparkMaxConfig climberConfig = createClimberConfig();
    climbMotor.configure(
        climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climbMotor.getEncoder().setPosition(0);
  }

  private SparkMaxConfig createClimberConfig() {
    SparkMaxConfig climberConfig = new SparkMaxConfig();
    climberConfig.encoder.positionConversionFactor(
        ClimberConstants.climberPCF); // Rot to deg conversion

    climberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    climberConfig.smartCurrentLimit(ClimberConstants.climberCurrentLimit).idleMode(IdleMode.kBrake);
    climberConfig
        .softLimit
        .forwardSoftLimit(ClimberConstants.climberSoftLimit)
        .forwardSoftLimitEnabled(true);

    return climberConfig;
  }

  public Command createClimbOutCommand() {
    // TODO: make sure 1 is correct direction
    return Commands.startEnd(
        () -> climbMotor.set(-0.1),
        // Stop the climber at the end of the command
        () -> climbMotor.set(0.0),
        this);
  }

  public Command createClimbInCommand() {
    return Commands.startEnd(() -> climbMotor.set(0.1), () -> climbMotor.set(0.0), this);
  }
}
