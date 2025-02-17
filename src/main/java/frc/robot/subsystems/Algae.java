// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

final class AlgaeConstants {
  static final int algaeBendMotorID = 3;
  static final double algaeBendPCF = 12.8; // check what value the PCF should actually be
  static final int algaeBendCurrentLimit = 0;
  static final int algaeIntakeMotorID = 4;
  static final double bendSoftLimit = 0;
  static final double algaeIntakePosition = 72;

  static final double bendP = 1;
  static final double bendI = 0;
  static final double bendD = 0;
}

public class Algae extends SubsystemBase {
  private SparkMax algaeBendMotor =
      new SparkMax(AlgaeConstants.algaeBendMotorID, MotorType.kBrushless);
  private SparkMax algaeIntakeMotor =
      new SparkMax(AlgaeConstants.algaeIntakeMotorID, MotorType.kBrushed);

  public enum bendState {
    UP,
    DOWN
  }

  public Algae() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Algae position", algaeBendMotor.getEncoder().getPosition());
  }

  private SparkMaxConfig algaeBendConfig(boolean coast) {
    SparkMaxConfig algaeBendConfig = new SparkMaxConfig();
    algaeBendConfig
        .encoder
        .positionConversionFactor(AlgaeConstants.algaeBendPCF) // Rotations to degrees conversion
        .velocityConversionFactor(AlgaeConstants.algaeBendPCF / 60.0d); // RPM to deg/s conversion

    algaeBendConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(AlgaeConstants.bendP)
        .i(AlgaeConstants.bendI)
        .d(AlgaeConstants.bendD);

    algaeBendConfig
        .smartCurrentLimit(AlgaeConstants.algaeBendCurrentLimit)
        .idleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
    algaeBendConfig
        .softLimit
        .forwardSoftLimit(AlgaeConstants.bendSoftLimit)
        .forwardSoftLimitEnabled(true);

    return algaeBendConfig;
  }

  // TODO change intake and output brushless motors to PID
  public Command intakeAlgae() {
    return Commands.startEnd(
        () ->
            algaeBendMotor
                .getClosedLoopController()
                .setReference(
                    AlgaeConstants.algaeIntakePosition, ControlType.kMAXMotionPositionControl),
        () ->
            algaeBendMotor.configure(
                algaeBendConfig(true),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters));
  }

  public Command outputAlgae() {
    return Commands.startEnd(
        () ->
            algaeBendMotor
                .getClosedLoopController()
                .setReference(AlgaeConstants.algaeIntakePosition, ControlType.kPosition),
        () ->
            algaeBendMotor.configure(
                algaeBendConfig(false),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters));
  }

  // TODO LOOK AT VALUES
  public Command brushedIntake() {
    return Commands.startEnd(() -> algaeIntakeMotor.set(1), () -> algaeIntakeMotor.set(0.0), this);
  }

  public Command outputBrushed() {
    return Commands.startEnd(() -> algaeIntakeMotor.set(-1), () -> algaeIntakeMotor.set(0.0), this);
  }
}
