// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

final class CoralScorerConstants {
  static final int intakePhotoSensorDIOPin = 9;
  static final int motorID = 51;
  static final int currentLimit = 40;
  static final boolean inverted = false;
}

public class CoralScorer extends SubsystemBase {
  SparkMax effector;

  private final DigitalInput photoSensor;

  public CoralScorer() {
    photoSensor = new DigitalInput(CoralScorerConstants.intakePhotoSensorDIOPin);
    effector = new SparkMax(CoralScorerConstants.motorID, MotorType.kBrushless);

    SparkMaxConfig effectorConfig = effectorConfig();

    effector.configure(
        effectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    return !photoSensor.get();
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

  public void effectorReverse() {
    effector.set(-1.0);
  }

  public Command reverseCommand() {
    return Commands.startEnd(this::effectorReverse, this::effectorStop, this);
  }

  public Command intakeCMD() {
    // Runs End Effector forward until game piece detected, then stops it
    return Commands.run(this::effectorForward, this)
        .finallyDo(this::effectorStop)
        .until(this::hasGamePiece);
  }

  public Command manualIntakeCMD() {
    return Commands.run(this::effectorSlowForward, this)
        .finallyDo(this::effectorStop)
        .withTimeout(1);
  }

  public Command depositCMD() {
    return Commands.run(this::effectorForward, this).finallyDo(this::effectorStop);
  }

  public Command slowDepositCMD() {
    return Commands.run(this::effectorSlowForward, this).finallyDo(this::effectorStop);
  }
}
