// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


class climberConstants {

  static double climberPCF = 0;

  static int climberCurrentLimit = 0;
  
  static int climbMotorID= 3;
}

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public SparkMax climbMotor = new SparkMax(climberConstants.climbMotorID,MotorType.kBrushless);
 //variable for the subsystem


  
  public Climber() {
    SparkMaxConfig climberConfig = createClimberConfig();
    climbMotor.configure(climberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  private SparkMaxConfig createClimberConfig() {
        SparkMaxConfig climberConfig = new SparkMaxConfig();
        climberConfig.encoder
                .positionConversionFactor(climberConstants.climberPCF)
                .velocityConversionFactor(climberConstants.climberPCF / 60.0d);

        climberConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        climberConfig.smartCurrentLimit(climberConstants.climberCurrentLimit).idleMode(IdleMode.kBrake);

        return climberConfig;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    public Command createClimbCommand(double dutyCycle) {
    return Commands.startEnd(
      () -> climbMotor.set(dutyCycle),
      // Stop the flywheel at the end of the command
      () -> climbMotor.set(0.0),
      this
);

  }
}
