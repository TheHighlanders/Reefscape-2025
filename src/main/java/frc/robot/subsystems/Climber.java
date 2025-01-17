// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
class moduleConstants {
  static double angleP = 0;
  static double angleI = 0;
  static double angleD = 0;

  static double driveP = 0;
  static double driveI = 0;
  static double driveD = 0;

  static double driveS = 0;
  static double driveV = 0;
  static double driveA = 0;

  static double drivePCF = Units.inchesToMeters(4) * Math.PI / 8.14d;
  static double anglePCF = 360.0 / 12.8d;

  static int driveCurrentLimit = 0;
  static int angleCurrentLimit = 0;

  static boolean driveInverted = false;
  static boolean angleInverted = false;
  static boolean absolInverted = false;

  static double maxSpeed = Constants.maxSpeed;
}
public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public SparkMax climbMotor = new SparkMax(Constants.climberConstants.climbMotorID,MotorType.kBrushless);
  public Climber() {
    SparkMaxConfig climberConfig = createClimberConfig();
  }
 private SparkMaxConfig createClimberConfig() {
        SparkMaxConfig climberConfig = new SparkMaxConfig();
        climberConfig.encoder
                .positionConversionFactor(moduleConstants.drivePCF)
                .velocityConversionFactor(moduleConstants.drivePCF / 60.0d);

        climberConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(moduleConstants.driveP, moduleConstants.driveI, moduleConstants.driveD);

        climberConfig.smartCurrentLimit(moduleConstants.driveCurrentLimit).idleMode(IdleMode.kBrake);

        return climberConfig;
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
