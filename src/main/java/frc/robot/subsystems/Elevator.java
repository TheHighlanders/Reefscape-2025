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
import com.revrobotics.spark.config.ClosedLoopConfigAccessor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

class elevatorConstants {
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

  double arbFF;

  private ElevatorState uppydowny = ElevatorState.HOME;

  public Elevator() { // Creates a new Elevator.
    elevatorMotorConfig = new SparkMaxConfig();
    elevatorMotor = new SparkMax(elevatorConstants.elevMotorID, MotorType.kBrushless);
    reverseLimitSwitch = elevatorMotor.getReverseLimitSwitch();

    elevatorMotorConfig
        .encoder
        .positionConversionFactor(elevatorConstants.elevPCF)
        .velocityConversionFactor(elevatorConstants.elevPCF / 60.0d);

    elevatorController = elevatorMotor.getClosedLoopController();
    elevatorMotorConfig.idleMode(IdleMode.kBrake);

    elevatorMotorConfig
        .limitSwitch
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

    elevatorMotorConfig
        .softLimit
        .forwardSoftLimit(elevatorConstants.forwardSoftLimit) // TODO: chanege limit value
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(elevatorConstants.backwardSoftLimit)
        .reverseSoftLimitEnabled(true);

    // Set Smart Motion and Smart Velocity parameters.
    elevatorMotorConfig
        .closedLoop
        .maxMotion
        .maxVelocity(elevatorConstants.maxVelocity) // TODO find these desirerd values
        .maxAcceleration(elevatorConstants.maxAccel)
        .allowedClosedLoopError(elevatorConstants.maxClosedLoopError);
    // Set PID gains
    elevatorMotorConfig
        .closedLoop // pid loop to control elevator elevating rate
        .p(elevatorConstants.elevP)
        .i(elevatorConstants.elevI) // TODO find these desirerd values
        .d(elevatorConstants.elevD)
        .outputRange(elevatorConstants.outputRange[0], elevatorConstants.outputRange[1]);

    elevatorMotor.configure(
        elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // TODO: Encoder should be configured (position conversion factor), based on cad, in order to
    // use the feedback
    elevatorEncoder = elevatorMotor.getEncoder();

    arbFF = elevatorConstants.feedFoward;

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
        targetPosition = elevatorConstants.homeTarget; // the set point
        break;
      case L1_POSITION:
        targetPosition = elevatorConstants.l1Target; // the set point
        break;
      case L2_POSITION:
        targetPosition = elevatorConstants.l2Target; // the set point
        break;
      case L3_POSITION:
        targetPosition = elevatorConstants.l3Target; // the set point
        break;
      case L4_POSITION:
        targetPosition = elevatorConstants.l4Target; // the set point
        break;
      case CORAL_POSITION:
        targetPosition = elevatorConstants.coralPositionTarget; // the set point
        break;
    }
    elevatorController.setReference(
        targetPosition, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, arbFF);
  }

  public void L1() {
    uppydowny = ElevatorState.L1_POSITION;
  }

  public void L2() {
    uppydowny = ElevatorState.L2_POSITION;
  }

  public void L3() {
    uppydowny = ElevatorState.L3_POSITION;
  }

  public void L4() {
    uppydowny = ElevatorState.L4_POSITION;
  }

  public void HOME() {
    uppydowny = ElevatorState.HOME;
  }

  public void coralPosition() {
    uppydowny = ElevatorState.CORAL_POSITION;
  }

  public Command getL1Command() {
    return new InstantCommand(this::L1);
  }

  public Command getL2Command() {
    return new InstantCommand(this::L2);
  }

  public Command getL3Command() {
    return new InstantCommand(this::L3);
  }

  public Command getL4Command() {
    return new InstantCommand(this::L4);
  }

  public Command getCoralPositionCommand() {
    return new InstantCommand(this::coralPosition);
  }

  public void sendTuningConstants() {
    ClosedLoopConfigAccessor config = elevatorMotor.configAccessor.closedLoop;
    SmartDashboard.putNumber("Tuning/Elevator/Elevator P", config.getP());
    SmartDashboard.putNumber("Tuning/Elevator/Elevator I", config.getI());
    SmartDashboard.putNumber("Tuning/Elevator/Elevator D", config.getD());
    SmartDashboard.putNumber("Tuning/Elevator/Elevator F", config.getFF());

    SmartDashboard.putNumber("Tuning/Elevator/Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber(
        "Tuning/Elevator/Error", targetPosition - elevatorEncoder.getPosition());
  }

  public void updateTuningConstants() {

    double p = SmartDashboard.getNumber("Tuning/Elevator/Elevator P", elevatorConstants.elevP);
    double i = SmartDashboard.getNumber("Tuning/Elevator/Elevator I", elevatorConstants.elevI);
    double d = SmartDashboard.getNumber("Tuning/Elevator/Elevator D", elevatorConstants.elevD);
    double f = SmartDashboard.getNumber("Tuning/Elevator/Elevator F", elevatorConstants.feedFoward);

    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(p, i, d);
    arbFF = f;
    elevatorMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
