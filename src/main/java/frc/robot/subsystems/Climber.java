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


import edu.wpi.first.wpilibj2.command.SubsystemBase;


class climberConstants {

  static double climberP = 0;
  static double climberI = 0;
  static double climberD = 0;

  static double climberS = 0;
  static double climerV = 0;
  static double climberA = 0;

  static double climberPCF = 0;

  static int climberCurrentLimit = 0;
  
  static int climbMotorID= 0;
}

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public SparkMax climbMotor = new SparkMax(climberConstants.climbMotorID,MotorType.kBrushless);
  
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
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(climberConstants.climberP, climberConstants.climberI, climberConstants.climberD);

        climberConfig.smartCurrentLimit(climberConstants.climberCurrentLimit).idleMode(IdleMode.kBrake);

        return climberConfig;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
