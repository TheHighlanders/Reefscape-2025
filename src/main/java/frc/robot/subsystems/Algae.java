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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

final class AlgaeConstants {

  static final int algaeBendMotorID = 3;
  static final double algaeBendPCF = 12.8; // check what value the PCF should actually be
  static final int algaeBendCurrentLimit = 20;
  static final int algaeIntakeMotorID = 4;
  static final double bendSoftLimit = 20;

  static final double algaeIntakePosition = -25;
  static final double algaeDislodgePosition = -1; // TODO: find this

  static final double bendP = 10;
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

    if (Constants.devMode) {
      SmartDashboard.putNumber("Algae position", algaeBendMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("algae current", algaeBendMotor.getAppliedOutput());
    }
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

    algaeBendMotor.getEncoder().setPosition(0);

    return algaeBendConfig;
  }

  private Command toPosition(double position) {
    return Commands.runOnce(
            () ->
                algaeBendMotor
                    .getClosedLoopController()
                    .setReference(position, ControlType.kPosition))
        .withName("Move Algae to Position " + position);
  }

  // TODO change intake and output brushless motors to PID
  public Command homePosition() {
    return toPosition(0).withName("Algae Home Position");
  }

  public Command processorPosition() {
    return toPosition(AlgaeConstants.algaeIntakePosition)
        .until(
            () ->
                MathUtil.isNear(
                    AlgaeConstants.algaeIntakePosition,
                    algaeBendMotor.getEncoder().getPosition(),
                    0.1))
        .finallyDo(
            () ->
                algaeBendMotor.configure(
                    algaeBendConfig(false),
                    SparkBase.ResetMode.kResetSafeParameters,
                    SparkBase.PersistMode.kPersistParameters))
        .withName("Algae Processor Position");
  }

  public Command dislodgePosition() {
    return toPosition(AlgaeConstants.algaeDislodgePosition).withName("Algae Dislodge Position");
  }

  // TODO LOOK AT VALUES
  public Command intakeAlgae() {
    return Commands.startEnd(() -> algaeIntakeMotor.set(1), () -> algaeIntakeMotor.set(0.0), this)
        .withName("Intake Algae");
  }

  public Command outputAlgae() {
    return Commands.startEnd(() -> algaeIntakeMotor.set(-1), () -> algaeIntakeMotor.set(0.0), this)
        .withName("Output Algae");
  }
}
