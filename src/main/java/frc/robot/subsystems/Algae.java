// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

final class algaeConstants {
  static final int algaeBendMotorID = 4;
  static final double algaeBendPCF = 12.8; // check what value the PCF should actually be
  static final int algaeBendCurrentLimit = 0;
  static final int algaeSpinMotorID = 5;
  static final double bendSoftLimit = 0;
}

public class Algae extends SubsystemBase {

  private SparkMax algaeBendMotor =
      new SparkMax(algaeConstants.algaeBendMotorID, MotorType.kBrushless);
  private SparkMax algaeSpinMotor =
      new SparkMax(algaeConstants.algaeSpinMotorID, MotorType.kBrushed);

  public Algae() {}

  private SparkMaxConfig createAlgaeBendConfig() {
    SparkMaxConfig algaeBendConfig = new SparkMaxConfig();
    algaeBendConfig
        .encoder
        .positionConversionFactor(algaeConstants.algaeBendPCF) // Rotations to degrees conversion
        .velocityConversionFactor(algaeConstants.algaeBendPCF / 60.0d); // RPM to deg/s conversion

    algaeBendConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    algaeBendConfig
        .smartCurrentLimit(algaeConstants.algaeBendCurrentLimit)
        .idleMode(IdleMode.kCoast);
    algaeBendConfig
        .softLimit
        .forwardSoftLimit(algaeConstants.bendSoftLimit)
        .forwardSoftLimitEnabled(true);

    return algaeBendConfig;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
