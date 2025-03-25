// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Elevator.ElevatorState;

final class CoralScorerConstants {

  static final int intakeBeamBreakDIOPin = 9;
  static final int motorID = 51;
  static final int currentLimit = 40;
  static final boolean inverted = false;

  static final Map<ElevatorState, Double> heightToSpeedMap = new HashMap<>();

  static {
    heightToSpeedMap.put(ElevatorState.HOME, 0.3);
    heightToSpeedMap.put(ElevatorState.L2_POSITION, 0.7);
    heightToSpeedMap.put(ElevatorState.L3_POSITION, 0.7);
    heightToSpeedMap.put(ElevatorState.L4_POSITION, 0.7);
  }
}

public class CoralScorer extends SubsystemBase {

  SparkMax effector;

  private final DigitalInput beamBreak;

  public CoralScorer() {
    beamBreak = new DigitalInput(CoralScorerConstants.intakeBeamBreakDIOPin);
    effector = new SparkMax(CoralScorerConstants.motorID, MotorType.kBrushless);

    SparkMaxConfig effectorConfig = effectorConfig();

    effector.configure(
        effectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.setName("Coral");
  }

  private SparkMaxConfig effectorConfig() {
    SparkMaxConfig effectorConfig = new SparkMaxConfig();

    effectorConfig.inverted(CoralScorerConstants.inverted);
    effectorConfig.idleMode(IdleMode.kBrake);

    effectorConfig.smartCurrentLimit(CoralScorerConstants.currentLimit).idleMode(IdleMode.kCoast);

    return effectorConfig;
  }

  @Override
  public void periodic() {}

  public boolean hasGamePiece() {
    return !beamBreak.get();
  }

  public void effectorStop() {
    effector.set(0);
  }

  public void effectorForward() {
    effector.set(1.0);
  }

  public void effectorSlowForward() {
    effector.set(0.3);
  }

  public void effectorSpeedByHeight(ElevatorState height) {
    effector.set(CoralScorerConstants.heightToSpeedMap.get(height));
  }

  public void effectorReverse() {
    effector.set(-1.0);
  }

  public Command reverseCommand() {
    return Commands.startEnd(this::effectorReverse, this::effectorStop, this)
        .withName("Reverse Coral Effector");
  }

  public Command intakeCMD() {
    // Runs End Effector forward until game piece detected, then stops it
    return Commands.run(this::effectorForward, this)
        .finallyDo(this::effectorStop)
        .until(this::hasGamePiece)
        .withName("Intake Coral Until Detected");
  }

  public Command manualIntakeCMD() {
    return Commands.run(this::effectorSlowForward, this)
        .finallyDo(this::effectorStop)
        .withTimeout(1)
        .withName("Manual Slow Intake");
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

  public Command depositByHeightCMD(ElevatorState height) {
    SmartDashboard.putNumber(
        "Coral/Deposit speed", CoralScorerConstants.heightToSpeedMap.get(height));
    return Commands.run(() -> this.effectorSpeedByHeight(height), this);
  }
}
