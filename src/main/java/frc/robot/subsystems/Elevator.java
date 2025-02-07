// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

final class ElevatorConstants {
  static final int elevMotorID = 40;

  static final double elevPCF = 1;

  static final double homeTarget = 0;
  static final double l1Target = 0;
  static final double l2Target = 10;
  static final double l3Target = 20;
  static final double l4Target = 30;
  static final double coralPositionTarget = 40;

  static final double feedFoward = 8;
  static final double elevP = 1;
  static final double elevI = 0;
  static final double elevD = 0;

  static final double forwardSoftLimit = 50;
  static final double backwardSoftLimit = 0;

  static final double[] outputRange = {-1d, 1d};

  static final double maxVelocity = 5;
  static final double maxAccel = 5;
  static final double maxClosedLoopError = 5;
}

public class Elevator extends SubsystemBase {
  public enum ElevatorState {
    HOME,
    L1_POSITION,
    L2_POSITION,
    L3_POSITION,
    L4_POSITION,
    CORAL_POSITION
  }

  private SparkMax elevatorMotor;
  private SparkMaxConfig elevatorMotorConfig;
  private SparkLimitSwitch reverseLimitSwitch;
  private RelativeEncoder elevatorEncoder;

  SparkClosedLoopController elevatorController;

  double targetPosition;

  private ElevatorState uppydowny = ElevatorState.HOME;

  public Elevator() { // Creates a new Elevator.
    elevatorMotorConfig = new SparkMaxConfig();
    elevatorMotor = new SparkMax(ElevatorConstants.elevMotorID, MotorType.kBrushless);
    reverseLimitSwitch = elevatorMotor.getReverseLimitSwitch();

    elevatorMotorConfig
        .encoder
        .positionConversionFactor(ElevatorConstants.elevPCF)
        .velocityConversionFactor(ElevatorConstants.elevPCF / 60.0d);

    elevatorController = elevatorMotor.getClosedLoopController();
    elevatorMotorConfig.idleMode(IdleMode.kBrake);

    elevatorMotorConfig
        .limitSwitch
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

    elevatorMotorConfig
        .softLimit
        .forwardSoftLimit(ElevatorConstants.forwardSoftLimit) // TODO: chanege limit value
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(ElevatorConstants.backwardSoftLimit)
        .reverseSoftLimitEnabled(true);

    SparkMaxConfig config = new SparkMaxConfig();

    // Set Smart Motion and Smart Velocity parameters.
    elevatorMotorConfig
        .closedLoop
        .maxMotion
        .maxVelocity(ElevatorConstants.maxVelocity) // TODO find these desirerd values
        .maxAcceleration(ElevatorConstants.maxAccel)
        .allowedClosedLoopError(ElevatorConstants.maxClosedLoopError);
    // Set PID gains
    config
        .closedLoop // pid loop to control elevator elevating rate
        .p(ElevatorConstants.elevP)
        .i(ElevatorConstants.elevI) // TODO find these desirerd values
        .d(ElevatorConstants.elevD)
        .outputRange(ElevatorConstants.outputRange[0], ElevatorConstants.outputRange[1]);

    elevatorMotor.configure(
        elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // TODO: Encoder should be configured (position conversion factor), based on
    // cad, in order to
    // use the feedback
    elevatorEncoder = elevatorMotor.getEncoder();

    // Reset the position to 0 to start within the range of the soft limits
    elevatorEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (reverseLimitSwitch.isPressed()) { // when the switch is pressed stop the motor
      elevatorEncoder.setPosition(0);
    }

    switch (uppydowny) {
      case HOME:
        targetPosition = ElevatorConstants.homeTarget; // the set point
        break;
      case L1_POSITION:
        targetPosition = ElevatorConstants.l1Target; // the set point
        break;
      case L2_POSITION:
        targetPosition = ElevatorConstants.l2Target; // the set point
        break;
      case L3_POSITION:
        targetPosition = ElevatorConstants.l3Target; // the set point
        break;
      case L4_POSITION:
        targetPosition = ElevatorConstants.l4Target; // the set point
        break;
      case CORAL_POSITION:
        targetPosition = ElevatorConstants.coralPositionTarget; // the set point
        break;
    }
    elevatorController.setReference(
        targetPosition,
        ControlType.kMAXMotionPositionControl,
        ClosedLoopSlot.kSlot0,
        ElevatorConstants.feedFoward);
  }

  public Command setPosition(ElevatorState position) {
    return Commands.runOnce(() -> uppydowny = position);
  }
}
