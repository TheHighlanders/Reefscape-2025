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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

final class algaeConstants {
  static final int algaeBendMotorID = 4;
  static final double algaeBendPCF = 12.8; // check what value the PCF should actually be
  static final int algaeBendCurrentLimit = 0;
  static final int algaeIntakeMotorID = 5;
  static final double bendSoftLimit = 0;
}

public class Algae extends SubsystemBase {
  private SparkMax algaeBendMotor =
      new SparkMax(algaeConstants.algaeBendMotorID, MotorType.kBrushless);
  private SparkMax algaeIntakeMotor =
      new SparkMax(algaeConstants.algaeIntakeMotorID, MotorType.kBrushed);

  public enum bendState {
    UP,
    DOWN
  }

  public Algae() {}

  private SparkMaxConfig algaeBendConfig(boolean coast) {
    SparkMaxConfig algaeBendConfig = new SparkMaxConfig();
    algaeBendConfig
        .encoder
        .positionConversionFactor(algaeConstants.algaeBendPCF) // Rotations to degrees conversion
        .velocityConversionFactor(algaeConstants.algaeBendPCF / 60.0d); // RPM to deg/s conversion

    algaeBendConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    algaeBendConfig
        .smartCurrentLimit(algaeConstants.algaeBendCurrentLimit)
        .idleMode(coast ? IdleMode.kCoast : IdleMode.kBrake);
    algaeBendConfig
        .softLimit
        .forwardSoftLimit(algaeConstants.bendSoftLimit)
        .forwardSoftLimitEnabled(true);

    return algaeBendConfig;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
//TODO change intake and output brushless motors to PID
  public Command intakeAlgae() {
    return Commands.parallel(
        Commands.runOnce(() -> algaeIntakeMotor.set(1)),
        Commands.sequence(
            Commands.runOnce(
                () ->
                    algaeBendMotor
                        .getClosedLoopController()
                        .setReference(72, ControlType.kMAXMotionPositionControl)),
            Commands.waitSeconds(1),
            Commands.runOnce(
                () ->
                    algaeBendMotor.configure(
                        algaeBendConfig(true),
                        SparkBase.ResetMode.kResetSafeParameters,
                        SparkBase.PersistMode.kPersistParameters))));
  }
  public Command outputAlgae() {
    return Commands.parallel(
      Commands.runOnce(() -> algaeIntakeMotor.set(-1)),
      Commands.sequence(
          Commands.runOnce(
              () ->
                  algaeBendMotor
                      .getClosedLoopController()
                      .setReference(72, ControlType.kPosition)),
          Commands.waitSeconds(1),
          Commands.runOnce(
              () ->
                  algaeBendMotor.configure(
                      algaeBendConfig(false),
                      SparkBase.ResetMode.kResetSafeParameters,
                      SparkBase.PersistMode.kPersistParameters))));
  }
  //TODO use whiletrue or finallydo and make brushed motor controls
public Command brushedIntake() {

}).finallyDo(()-> stop ());
}
public Command outputBrushed() {

}
 
}
