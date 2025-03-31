// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Set;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.Elevator.ElevatorState;

final class CoralScorerConstants {

  static final int intakeBeamBreakDIOPin = 0;
  static final int motorID = 51;
  static final int currentLimit = 40;
  static final boolean inverted = false;

  static final double bitePosition = 4000;
  static final double coralLengh = 12;
  static final double endThreshold = 10; // how far from the end to start going at specified speed
  static final double effectorPCF = 1 / (10 * 3 * Math.PI);

  static final double kP = 0.1;
  static final double kI = 0.0;
  static final double kD = 0.0;
  static final double kF = 0.0;
  static final double positionTolerance = 0.1;

  static final EnumMap<ElevatorState, Double> heightToSpeedMap = new EnumMap<>(ElevatorState.class);

  static {
    heightToSpeedMap.put(ElevatorState.HOME, 0.3d);
    heightToSpeedMap.put(ElevatorState.L2_POSITION, 1d);
    heightToSpeedMap.put(ElevatorState.L3_POSITION, 1d);
    heightToSpeedMap.put(ElevatorState.L4_POSITION, 0.3d);
  }
}

public class CoralScorer extends SubsystemBase {

  SparkMax effector;
  private final RelativeEncoder encoder;
  private SparkClosedLoopController coralController;

  private final DigitalInput beamBreak;

  public Trigger hasCoral;

  public CoralScorer() {
    beamBreak = new DigitalInput(CoralScorerConstants.intakeBeamBreakDIOPin);
    effector = new SparkMax(CoralScorerConstants.motorID, MotorType.kBrushless);

    SparkMaxConfig effectorConfig = effectorConfig();

    effector.configure(
        effectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = effector.getEncoder();
    coralController = effector.getClosedLoopController();

    hasCoral = new Trigger(this::hasGamePiece);

    this.setName("Coral");
  }

  private SparkMaxConfig effectorConfig() {
    SparkMaxConfig effectorConfig = new SparkMaxConfig();

    effectorConfig.inverted(CoralScorerConstants.inverted);
    effectorConfig.idleMode(IdleMode.kBrake);

    effectorConfig.smartCurrentLimit(CoralScorerConstants.currentLimit).idleMode(IdleMode.kCoast);

    effectorConfig
        .encoder
        .positionConversionFactor(CoralScorerConstants.effectorPCF)
        .velocityConversionFactor(CoralScorerConstants.effectorPCF / 60d);

    effectorConfig
        .closedLoop // pid loop to control elevator elevating rate
        .pidf(
            CoralScorerConstants.kP,
            CoralScorerConstants.kI,
            CoralScorerConstants.kD,
            CoralScorerConstants.kF,
            ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1);

    return effectorConfig;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral/EncoderPosition", encoder.getPosition());
    SmartDashboard.putBoolean("Coral/Has Piece", hasGamePiece());
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  private boolean atBitePosition() {
    return MathUtil.isNear(
        CoralScorerConstants.bitePosition,
        encoder.getPosition(),
        CoralScorerConstants.positionTolerance);
  }

  private boolean atEndTheshold() {
    return MathUtil.isNear(
        CoralScorerConstants.endThreshold,
        encoder.getPosition(),
        CoralScorerConstants.positionTolerance);
  }

  public Command resetEncoderCMD() {
    return Commands.runOnce(() -> encoder.setPosition(0));
  }

  public void setPosition(double position) {
    coralController.setReference(position, SparkBase.ControlType.kPosition);
  }

  public void setVelocity(double velocity) {
    coralController.setReference(-velocity, SparkBase.ControlType.kVelocity);
  }

  public void setDutyCycle(double dutyCycle) {
    coralController.setReference(dutyCycle, SparkBase.ControlType.kDutyCycle);
  }

  public void setBiteDutyCycle() {
    setDutyCycle(0.5);
  }

  public void effectorSpeedByHeight(ElevatorState height) {
    this.setDutyCycle(CoralScorerConstants.heightToSpeedMap.get(height));
  }

  public boolean hasGamePiece() {
    return !beamBreak.get();
  }

  public boolean doesNotHaveGamePiece() {
    return !hasGamePiece();
  }

  private Command runUntilGamePiece() {
    return Commands.run(this::setBiteDutyCycle, this)
        .until(this::hasGamePiece)
        .finallyDo(this::effectorStop)
        .withName("Run Until Game Piece");
  }

  private Command runToBitePosition() {
    return Commands.runOnce(() -> setPosition(CoralScorerConstants.bitePosition));
  }

  public Command biteCMD() {
    return runUntilGamePiece();
    // .andThen(
    //     runToBitePosition());
  }

  private Command runToEndThresh() {
    return Commands.runOnce(() -> setPosition(CoralScorerConstants.endThreshold))
        .andThen(Commands.waitUntil(this::atEndTheshold));
  }

  private Command runAtElevatorHeight(ElevatorState height) {
    return Commands.runOnce(() -> effectorSpeedByHeight(height));
  }

  private Command deferDeposit(ElevatorState height) {
    return runAtElevatorHeight(height);
  }

  public Command depositCMD(ElevatorState height) {
    return Commands.defer(
        () -> deferDeposit(height).until(hasCoral.negate()).andThen(Commands.waitSeconds(0.1)),
        Set.of(this));
  }

  // Manual stuff
  public Command manualIntakeCMD() {
    return Commands.run(this::effectorSlowForward, this)
        .finallyDo(this::effectorStop)
        .withTimeout(1)
        .withName("Manual Slow Intake");
  }

  public Command reverseCommand() {
    return Commands.startEnd(this::effectorReverse, this::effectorStop, this)
        .withName("Reverse Coral Effector");
  }

  public Command depositCMD() {
    return Commands.run(this::effectorForward, this)
        .finallyDo(this::effectorStop)
        .withName("Deposit Coral");
  }

  public Command slowDepositCMD() {
    return Commands.run(this::effectorSlowForward, this)
        .finallyDo(this::effectorStop)
        .withName("Slow Deposit Coral");
  }

  public Command intakeCMD() {
    return Commands.runOnce(this::effectorReverse);
  }

  public void effectorStop() {
    setPosition(encoder.getPosition());
  }

  public void effectorForward() {
    effector.set(1.0);
  }

  public void effectorReverse() {
    effector.set(-1.0);
  }

  public void effectorSlowForward() {
    effector.set(0.3);
  }
}
