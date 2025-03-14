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
import java.util.function.DoubleSupplier;

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
  static final double algaeHigh = 25;

  static final double coralBetweenReefOffset = 2;

  static final double antiSlamVoltageOffset = 0.25; // Ignores ff
  static final double feedForward = 0.9;
  static final double elevPUp = 0.4; // P gain for upward movement
  static final double elevIUp = 0; // I gain for upward movement
  static final double elevDUp = 20; // D gain for upward movement
  static final double ffUp = 0.9; // Feed forward for upward movement

  static final double elevPDown = 0.3; // P gain for downward movement
  static final double elevIDown = 0; // I gain for downward movement
  static final double elevDDown = 25; // D gain for downward movement
  static final double ffDown = 0.5;

  static final double forwardSoftLimit = 55;
  static final double backwardSoftLimit = 0;

  static final double maxVelocity = 80 * 60.0d;
  static final double maxAccel = 3600;
  static final double maxClosedLoopError = 5;

  static final double POSITION_TOLERANCE = 0.5;
}

public class Elevator extends SubsystemBase {

  public enum ElevatorState {
    HOME,
    L2_POSITION,
    L3_POSITION,
    L4_POSITION,
    ALGAELOW,
    ALGAEHIGH,
    CURRENT
  }

  private final SparkMax elevatorMotor;

  private final SparkLimitSwitch reverseLimitSwitch;

  double targetPosition;
  double positionOffset;

  @Logged double trim;

  double arbFFDown;
  double arbFFUp;
  double antiSlamVoltageOffset;

  private ElevatorState uppydowny = ElevatorState.HOME;

  private final SparkClosedLoopController elevatorController;

  private final RelativeEncoder elevatorEncoder;

  // private double maxV = ElevatorConstants.maxVelocity;
  // private double maxA = ElevatorConstants.maxAccel;
  public Elevator() { // Creates a new Elevator.
    SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();
    elevatorMotor = new SparkMax(ElevatorConstants.elevMotorID, MotorType.kBrushless);
    reverseLimitSwitch = elevatorMotor.getReverseLimitSwitch();
    elevatorMotorConfig
        .encoder
        .positionConversionFactor(ElevatorConstants.elevPCF) // inches
        .velocityConversionFactor(ElevatorConstants.elevPCF / 60.0d); // inches per second

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

    // Configure slot 0 for upward movement
    elevatorMotorConfig.closedLoop.pidf(
        ElevatorConstants.elevPUp,
        ElevatorConstants.elevIUp,
        ElevatorConstants.elevDUp,
        0,
        ClosedLoopSlot.kSlot0);

    // Configure slot 1 for downward movement
    elevatorMotorConfig.closedLoop.pidf(
        ElevatorConstants.elevPDown,
        ElevatorConstants.elevIDown,
        ElevatorConstants.elevDDown,
        0,
        ClosedLoopSlot.kSlot1);

    elevatorMotor.configure(
        elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    arbFFUp = ElevatorConstants.ffUp;
    arbFFDown = ElevatorConstants.ffDown;
    antiSlamVoltageOffset = ElevatorConstants.antiSlamVoltageOffset;

    elevatorController = elevatorMotor.getClosedLoopController();
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorEncoder.setPosition(0);

    trim = 0;
  }

  @Override
  public void periodic() {

    if (reverseLimitSwitch.isPressed()
        || elevatorEncoder.getPosition() < 0) { // when the switch is pressed stop the motor
      elevatorEncoder.setPosition(0);
    }

    if (uppydowny == ElevatorState.HOME) {
      if (MathUtil.isNear(ElevatorConstants.homeTarget, elevatorEncoder.getPosition(), 0.5)
          && !reverseLimitSwitch.isPressed()) {
        elevatorController.setReference(
            antiSlamVoltageOffset, ControlType.kVoltage, ClosedLoopSlot.kSlot0, 0);

        DriverStation.reportWarning("IN AUTOLAND", false);
      }
    }

    SmartDashboard.putNumber("Tuning/Elevator/Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Tuning/Elevator/Trim", trim);

    if (Constants.devMode) {
      SmartDashboard.putNumber("Tuning/Elevator/Setpoint", targetPosition);
      SmartDashboard.putNumber("Tuning/Elevator/Output", elevatorMotor.getAppliedOutput());
      SmartDashboard.putNumber("Tuning/Elevator/Velocity", elevatorEncoder.getVelocity());

      SmartDashboard.putNumber(
          "Tuning/Elevator/Error", targetPosition - elevatorEncoder.getPosition());
    }
  }

  public Command zeroElevator() {
    return Commands.runOnce(() -> elevatorEncoder.setPosition(0)).ignoringDisable(true);
  }

  public Command setPosition(ElevatorState position) {
    return Commands.runOnce(
        () -> {
          if (position != ElevatorState.CURRENT) {
            uppydowny = position;
          }

          double currentPosition = elevatorEncoder.getPosition();

          targetPosition =
              switch (uppydowny) {
                case HOME -> ElevatorConstants.homeTarget;
                case L2_POSITION -> ElevatorConstants.l2Target + trim + positionOffset;
                case L3_POSITION -> ElevatorConstants.l3Target + trim + positionOffset;
                case L4_POSITION -> ElevatorConstants.l4Target + trim + positionOffset;
                case ALGAELOW -> ElevatorConstants.algaeLow + trim;
                case ALGAEHIGH -> ElevatorConstants.algaeHigh + trim;
                default -> ElevatorConstants.homeTarget;
              };

          boolean movingUp = targetPosition > currentPosition;

          ClosedLoopSlot slot = ClosedLoopSlot.kSlot0; // UP
          double arbFF = arbFFUp;

          if (!movingUp) { // DOWN
            slot = ClosedLoopSlot.kSlot1;
            arbFF = arbFFDown;
          }

          elevatorController.setReference(targetPosition, ControlType.kPosition, slot, arbFF);
        });
  }

  public void sendTuningConstants() {
    ClosedLoopConfigAccessor config = elevatorMotor.configAccessor.closedLoop;

    SmartDashboard.putNumber("Tuning/Elevator/Elevator P Up", config.getP(ClosedLoopSlot.kSlot0));
    SmartDashboard.putNumber("Tuning/Elevator/Elevator I Up", config.getI(ClosedLoopSlot.kSlot0));
    SmartDashboard.putNumber("Tuning/Elevator/Elevator D Up", config.getD(ClosedLoopSlot.kSlot0));
    SmartDashboard.putNumber("Tuning/Elevator/Elevator F Up", ElevatorConstants.ffUp);

    SmartDashboard.putNumber("Tuning/Elevator/Elevator P Down", config.getP(ClosedLoopSlot.kSlot1));
    SmartDashboard.putNumber("Tuning/Elevator/Elevator I Down", config.getI(ClosedLoopSlot.kSlot1));
    SmartDashboard.putNumber("Tuning/Elevator/Elevator D Down", config.getD(ClosedLoopSlot.kSlot1));
    SmartDashboard.putNumber("Tuning/Elevator/Elevator F Down", ElevatorConstants.ffDown);

    SmartDashboard.putNumber("Tuning/Elevator/Anti Slam Voltage", antiSlamVoltageOffset);

    SmartDashboard.putNumber("Tuning/Elevator/Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber(
        "Tuning/Elevator/Error", targetPosition - elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Tuning/Elevator/Setpoint", targetPosition);

    // SmartDashboard.putNumber("Tuning/Elevator/Max V", maxV);
    // SmartDashboard.putNumber("Tuning/Elevator/Max A", maxA);
  }

  public void updateTuningConstants() {

    double pUp =
        SmartDashboard.getNumber("Tuning/Elevator/Elevator P Up", ElevatorConstants.elevPUp);
    double iUp =
        SmartDashboard.getNumber("Tuning/Elevator/Elevator I Up", ElevatorConstants.elevIUp);
    double dUp =
        SmartDashboard.getNumber("Tuning/Elevator/Elevator D Up", ElevatorConstants.elevDUp);
    double fUp = SmartDashboard.getNumber("Tuning/Elevator/Elevator F Up", ElevatorConstants.ffUp);

    double pDown =
        SmartDashboard.getNumber("Tuning/Elevator/Elevator P Down", ElevatorConstants.elevPDown);
    double iDown =
        SmartDashboard.getNumber("Tuning/Elevator/Elevator I Down", ElevatorConstants.elevIDown);
    double dDown =
        SmartDashboard.getNumber("Tuning/Elevator/Elevator D Down", ElevatorConstants.elevDDown);
    double fDown =
        SmartDashboard.getNumber("Tuning/Elevator/Elevator F Down", ElevatorConstants.ffDown);

    // double mV = SmartDashboard.getNumber("Tuning/Elevator/Max V", maxV);
    // double mA = SmartDashboard.getNumber("Tuning/Elevator/Max A", maxA);
    antiSlamVoltageOffset =
        SmartDashboard.getNumber(
            "Tuning/Elevator/Anti Slam Voltage", ElevatorConstants.antiSlamVoltageOffset);

    SparkMaxConfig upConfig = new SparkMaxConfig();
    upConfig.closedLoop.pidf(pUp, iUp, dUp, 0, ClosedLoopSlot.kSlot0);

    SparkMaxConfig downConfig = new SparkMaxConfig();
    downConfig.closedLoop.pidf(pDown, iDown, dDown, 0, ClosedLoopSlot.kSlot1);
    // config.closedLoop.maxMotion.maxAcceleration(mA);
    // config.closedLoop.maxMotion.maxVelocity(mV);
    // maxA = mA;
    // maxV = mV;

    arbFFDown = fDown;
    arbFFUp = fUp;

    elevatorMotor.configure(
        upConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    elevatorMotor.configure(
        downConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Logged(name = "Elevator Position", importance = Importance.INFO)
  public double getElevatorPosition() {
    return elevatorEncoder.getPosition();
  }

  public Command jogElevator(double voltage) {
    return Commands.run(
            () -> elevatorController.setReference(voltage + arbFFUp, ControlType.kVoltage))
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
        return setPosition(targetState)
            .alongWith(Commands.waitUntil(() -> isAtHome(ElevatorConstants.POSITION_TOLERANCE)));
      default:
        return setPosition(targetState)
            .alongWith(
                Commands.waitUntil(() -> isAtSetpoint(ElevatorConstants.POSITION_TOLERANCE)));
    }
  }

  public void trim(double trimAmount) {
    trim = -trimAmount * 2;
  }

  public Command trimCMD(DoubleSupplier trimSup) {
    return Commands.run(() -> trim(trimSup.getAsDouble()));
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

  public Command algaeCMD(DoubleSupplier algae) {
    return runOnce(
        () -> {
          if (algae.getAsDouble() <= -0.5) {
            setPosition(ElevatorState.ALGAEHIGH).schedule();
            ;
          } else if (algae.getAsDouble() >= 0.5) {
            setPosition(ElevatorState.ALGAELOW).schedule();
            ;
          }
          System.out.print(algae.getAsDouble());
        });
  }

  @Logged(name = "Elevator at Setpoint", importance = Importance.INFO)
  public boolean loggingElevatorSetpoint() {
    return isAtSetpoint(0.5);
  }
}
