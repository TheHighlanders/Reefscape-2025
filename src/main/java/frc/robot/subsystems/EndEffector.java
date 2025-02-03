// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// in this there will be a motor, a beam brake

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

class endEffectorConstants {
  static final int intakePhotoSensorDIOPin = 0;
  static final int motorID = 50;
  static final int currentLimit = 40;

  // (Radius * 2 * PI) / (10 to 1 gearing)
  static final double effectorPCF = (Units.inchesToMeters(3) * 2 * Math.PI) / 10;

  static final double effectP = 1;
  static final double effectI = 0;
  static final double effectD = 0;

  class simulation {
    static final double effectorMOI = 0.004;
  }
}

public class EndEffector extends SubsystemBase {
  /** Creates a new EndEffector. */
  SparkMax effector;

  SparkSim effectorSim;
  DCMotor effectorNeo;
  DCMotorSim effectorNeoSim;

  private DigitalInput photoSensor;

  public EndEffector() {
    photoSensor = new DigitalInput(endEffectorConstants.intakePhotoSensorDIOPin);
    effector = new SparkMax(endEffectorConstants.motorID, MotorType.kBrushless);
    effectorNeo = DCMotor.getNEO(1);
    effectorNeoSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                effectorNeo,
                endEffectorConstants.simulation.effectorMOI,
                endEffectorConstants.effectorPCF),
            effectorNeo);

    if (Constants.sim) {
      effectorSim = new SparkSim(effector, effectorNeo);
    }

    SparkMaxConfig effectorConfig = effectorConfg();

    effector.configure(
        effectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private SparkMaxConfig effectorConfg() {
    SparkMaxConfig angleConfig = new SparkMaxConfig();

    angleConfig.inverted(false);

    angleConfig
        .encoder
        .positionConversionFactor(endEffectorConstants.effectorPCF)
        .velocityConversionFactor(endEffectorConstants.effectorPCF / 60d);

    angleConfig
        .closedLoop
        .pid(
            endEffectorConstants.effectP,
            endEffectorConstants.effectI,
            endEffectorConstants.effectD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    angleConfig.smartCurrentLimit(endEffectorConstants.currentLimit).idleMode(IdleMode.kCoast);

    return angleConfig;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has Game Piece", hasGamePiece());
  }

  public boolean hasGamePiece() {
    return photoSensor.get();
  }

  public void intakeStop() {
    DriverStation.reportWarning("Intake stop", false);
    effector.set(0);
  }

  public void effectorForward() {
    DriverStation.reportWarning("Intake forward", false);
    effector.set(1.0);
  }

  public Command effectorForwardUntilBrakeCMD() {
    return new RunCommand(this::effectorForward, this)
      .finallyDo(this::intakeStop)
      .until(() -> !hasGamePiece());
  }

  public void effectorReverse() {
    DriverStation.reportWarning("Intake reverse", false);
    effector.set(-1.0);
  }
}
