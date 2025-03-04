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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

class ElevatorConstants {
  static final int elevMotorID = 41;

  // 2 for carriage movement relative to 1st stage, also includes DP, and gear
  // ratio
  static final double elevPCF = 2 * 1.751 / 9.0d * Math.PI;

  static final double homeTarget = 5; // Position before autolanding
  static final double l2Target = 11.5;
  static final double l3Target = 28;
  static final double l4Target = 52.125;
  static final double algaeLow = 7;

  static final double coralBetweenReefOffset = 2;

  static final double antiSlamVoltageOffset = 0.25; // Ignores ff
  static final double feedForward = 0.9;
  static final double elevP = 0.4;
  static final double elevI = 0;
  static final double elevD = 20;

  static final double forwardSoftLimit = 55;
  static final double backwardSoftLimit = 0;

  static final double maxVelocity = 80 * 60.0d;
  static final double maxAccel = 3600;
  static final double maxClosedLoopError = 5;
}

public class Elevator extends SubsystemBase {
  public enum ElevatorState {
    HOME,
    L2_POSITION,
    L3_POSITION,
    L4_POSITION,
    ALGAELOW
  }

  private SparkMax elevatorMotor;

  private SparkLimitSwitch reverseLimitSwitch;

  double targetPosition;
  double positionOffset;

  double arbFF;
  double antiSlamVoltageOffset;

  private ElevatorState uppydowny = ElevatorState.HOME;

  private SparkClosedLoopController elevatorController;

  private RelativeEncoder elevatorEncoder;

  private double maxV = ElevatorConstants.maxVelocity;
  private double maxA = ElevatorConstants.maxAccel;

  public Elevator() { // Creates a new Elevator.
    SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();
    elevatorMotor = new SparkMax(ElevatorConstants.elevMotorID, MotorType.kBrushless);
    reverseLimitSwitch = elevatorMotor.getReverseLimitSwitch();
    elevatorMotorConfig
        .encoder
        .positionConversionFactor(ElevatorConstants.elevPCF)
        .velocityConversionFactor(ElevatorConstants.elevPCF);

    elevatorMotorConfig.idleMode(IdleMode.kBrake).inverted(true);

    elevatorMotorConfig
        .limitSwitch
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

    elevatorMotorConfig
        .softLimit
        .forwardSoftLimit(ElevatorConstants.forwardSoftLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(ElevatorConstants.backwardSoftLimit)
        .reverseSoftLimitEnabled(true);

    elevatorMotorConfig.closedLoop.outputRange(-0.35, 0.85);

    elevatorMotorConfig.closedLoopRampRate(0.05);

    // Set PID gains
    elevatorMotorConfig
        .closedLoop // pid loop to control elevator elevating rate
        .p(ElevatorConstants.elevP)
        .i(ElevatorConstants.elevI) // TODO find these desirerd values
        .d(ElevatorConstants.elevD);

    elevatorMotor.configure(
        elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    arbFF = ElevatorConstants.feedForward;
    antiSlamVoltageOffset = ElevatorConstants.antiSlamVoltageOffset;

    // Reset the position to 0 to start within the range of the soft limits

    elevatorController = elevatorMotor.getClosedLoopController();

    elevatorEncoder = elevatorMotor.getEncoder();

    elevatorEncoder.setPosition(0);
  }

  @Override
  public void periodic() {

    if (reverseLimitSwitch.isPressed()) { // when the switch is pressed stop the motor
      elevatorEncoder.setPosition(0);
    }

    if (uppydowny == ElevatorState.HOME) {
      if (MathUtil.isNear(ElevatorConstants.homeTarget, elevatorEncoder.getPosition(), 0.5)
          && !reverseLimitSwitch.isPressed()) {
        // elevatorMotor.setVoltage(antiSlamVoltageOffset);
        elevatorController.setReference(
            antiSlamVoltageOffset, ControlType.kVoltage, ClosedLoopSlot.kSlot0, 0);

        DriverStation.reportWarning("IN AUTOLAND", false);
      }
    }

    if (elevatorEncoder.getPosition() < 0) {
      elevatorEncoder.setPosition(0);
    }

    SmartDashboard.putNumber("Tuning/Elevator/Position", elevatorEncoder.getPosition());

    if (Constants.devMode) {
      SmartDashboard.putNumber("Tuning/Elevator/VoltagekF", arbFF);
      SmartDashboard.putNumber("Tuning/Elevator/Setpoint", targetPosition);
      SmartDashboard.putNumber("Tuning/Elevator/Output", elevatorMotor.getAppliedOutput());
      SmartDashboard.putNumber("Tuning/Elevator/Velocity", elevatorEncoder.getVelocity());

      SmartDashboard.putNumber(
          "Tuning/Elevator/Error", targetPosition - elevatorEncoder.getPosition());
    }
  }

  public Command zeroElevator() {
    return Commands.runOnce(
            () -> {
              elevatorEncoder.setPosition(0);
            })
        .ignoringDisable(true);
  }

  public Command setPosition(ElevatorState position) {
    return Commands.runOnce(
        () -> {
          uppydowny = position;
          switch (uppydowny) {
            case HOME:
              targetPosition = ElevatorConstants.homeTarget;
              break;
            case L2_POSITION:
              targetPosition = ElevatorConstants.l2Target;
              break;
            case L3_POSITION:
              targetPosition = ElevatorConstants.l3Target;
              break;
            case L4_POSITION:
              targetPosition = ElevatorConstants.l4Target;
              break;
            case ALGAELOW:
              targetPosition = ElevatorConstants.algaeLow;
          }
          elevatorController.setReference(
              targetPosition + positionOffset, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFF);
        });
  }

  public void sendTuningConstants() {
    ClosedLoopConfigAccessor config = elevatorMotor.configAccessor.closedLoop;
    SmartDashboard.putNumber("Tuning/Elevator/Elevator P", config.getP());
    SmartDashboard.putNumber("Tuning/Elevator/Elevator I", config.getI());
    SmartDashboard.putNumber("Tuning/Elevator/Elevator D", config.getD());
    SmartDashboard.putNumber("Tuning/Elevator/Elevator F", arbFF);

    SmartDashboard.putNumber("Tuning/Elevator/Anti Slam Voltage", antiSlamVoltageOffset);

    SmartDashboard.putNumber("Tuning/Elevator/Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber(
        "Tuning/Elevator/Error", targetPosition - elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Tuning/Elevator/Setpoint", targetPosition);

    SmartDashboard.putNumber("Tuning/Elevator/Max V", maxV);
    SmartDashboard.putNumber("Tuning/Elevator/Max A", maxA);
  }

  public void updateTuningConstants() {

    double p = SmartDashboard.getNumber("Tuning/Elevator/Elevator P", ElevatorConstants.elevP);
    double i = SmartDashboard.getNumber("Tuning/Elevator/Elevator I", ElevatorConstants.elevI);
    double d = SmartDashboard.getNumber("Tuning/Elevator/Elevator D", ElevatorConstants.elevD);
    double f =
        SmartDashboard.getNumber("Tuning/Elevator/Elevator F", ElevatorConstants.feedForward);

    double mV = SmartDashboard.getNumber("Tuning/Elevator/Max V", maxV);
    double mA = SmartDashboard.getNumber("Tuning/Elevator/Max A", maxA);

    antiSlamVoltageOffset =
        SmartDashboard.getNumber(
            "Tuning/Elevator/Anti Slam Voltage", ElevatorConstants.antiSlamVoltageOffset);

    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(p, i, d);
    arbFF = f;
    config.closedLoop.maxMotion.maxAcceleration(mA);
    config.closedLoop.maxMotion.maxVelocity(mV);
    maxA = mA;
    maxV = mV;

    elevatorMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Logged(name = "Elevator Position", importance = Importance.INFO)
  public double getElevatorPosition() {
    return elevatorEncoder.getPosition();
  }

  public Command jogElevator(double voltage) {
    return Commands.run(
            () -> elevatorController.setReference(voltage + arbFF, ControlType.kVoltage))
        .finallyDo(() -> elevatorMotor.stopMotor());
  }

  public boolean isAtSetpoint(double tolerance) {
    return MathUtil.isNear(targetPosition, elevatorEncoder.getPosition(), tolerance);
  }

  public boolean isAtHome(double tolerance) {
    return MathUtil.isNear(0, elevatorEncoder.getPosition(), tolerance);
  }

  public Command elevatorAuto(ElevatorState targetState) {
    switch (targetState) {
      case HOME:
        return setPosition(targetState).alongWith(Commands.waitUntil(() -> isAtHome(0.5)));
      default:
        return setPosition(targetState).alongWith(Commands.waitUntil(() -> isAtSetpoint(0.5)));
    }
  }

  public Command offsetElevator() {
    return Commands.startEnd(
        () -> {
          positionOffset = ElevatorConstants.coralBetweenReefOffset;
          setPosition(uppydowny).schedule();
        },
        () -> {
          positionOffset = 0;
          setPosition(uppydowny).schedule();
        });
  }

  @Logged(name = "Elevator Target", importance = Importance.INFO)
  public double getTargetPosition() {
    return targetPosition;
  }

  @Logged(name = "Elevator at Home", importance = Importance.INFO)
  public boolean loggingElevatorHome() {
    return isAtHome(0.5);
  }

  @Logged(name = "Elevator at Setpoint", importance = Importance.INFO)
  public boolean loggingElevatorSetpoint() {
    return isAtSetpoint(0.5);
  }
}
