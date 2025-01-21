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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
  
  static int climbMotorID= 3;


  static double climber2P = 0;    //2nd motor's constants
  static double climber2I = 0;
  static double climber2D = 0;

  static double climber2S = 0;
  static double climer2V = 0;
  static double climber2A = 0;

  static double climber2PCF = 0;

  static int climber2CurrentLimit = 0;
  
  static int climb2MotorID= 4;
}

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public SparkMax climbMotor = new SparkMax(climberConstants.climbMotorID,MotorType.kBrushless);
  public SparkMax climb2Motor = new SparkMax(climberConstants.climbMotorID,MotorType.kBrushless);
 //variable for the subsystem
  public Climber m_Climber;

  
  public Climber() {
    SparkMaxConfig climberConfig = createClimberConfig();
    climbMotor.configure(climberConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig climber2Config = createClimber2Config();
    climb2Motor.configure(climber2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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

    private SparkMaxConfig createClimber2Config() {
      SparkMaxConfig climber2Config = new SparkMaxConfig();
      climber2Config.encoder
              .positionConversionFactor(climberConstants.climberPCF)        //climber contants means the variables at the top :)
              .velocityConversionFactor(climberConstants.climberPCF / 60.0d);

      climber2Config.closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .pid(climberConstants.climber2P, climberConstants.climber2I, climberConstants.climber2D);

      climber2Config.smartCurrentLimit(climberConstants.climber2CurrentLimit).idleMode(IdleMode.kBrake);

      return climber2Config;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command createMoveClimberCommand(double targetPosition) {
   return Commands.startEnd(
    // Start a flywheel spinning at 50% power
    () -> climbMotor.set(targetPosition),
    // Stop the flywheel at the end of the command
    () -> climbMotor.set(0.0),
    // Requires the shooter subsystem
    m_Climber
);

  }
}
