// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// PLACEHOLDER FOR SUPERSTRUCTURE EXAMPLE

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.subsystems.Superstructure.IntakeState;
// import frc.robot.utils.StateRequest;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {

  private SparkMax elevatorMotor;
  private SparkMaxConfig elevatorMotorConfig;
  private SparkLimitSwitch reverseLimitSwitch;
  private RelativeEncoder elevatorEncoder;

  SparkClosedLoopController m_controller;

  private double feedFoward = 8;

  /** Creates a new Elevator. */
  public enum ElevatorState {
    HOME,
    L1_POSITION,
    L2_POSITION,
    L3_POSITION,
    L4_POSITION,
    CORAL_POSITION
  }

  private ElevatorState uppydowny = ElevatorState.HOME;

  public Elevator() {

    m_controller = elevatorMotor.getClosedLoopController();

    elevatorMotor = new SparkMax(1, MotorType.kBrushless);
    reverseLimitSwitch = elevatorMotor.getReverseLimitSwitch();
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorMotorConfig = new SparkMaxConfig();
    elevatorMotorConfig.idleMode(IdleMode.kBrake);
    elevatorMotorConfig.limitSwitch
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

    elevatorMotorConfig.softLimit
        // .forwardSoftLimit(50)
        // .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true);

    SparkMaxConfig config = new SparkMaxConfig();

    // Set Smart Motion and Smart Velocity parameters.
    elevatorMotorConfig.closedLoop.maxMotion
        .maxVelocity(5) // TODO find these desigherd values
        .maxAcceleration(5)
        .allowedClosedLoopError(5);
    // Set PID gains
    config.closedLoop
        .p(.1)
        .i(.029) // TODO find these desigherd values
        .d(.01)
        .outputRange(4, 5);

    elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

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
        m_controller.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedFoward); // the first value is
                                                                                                // the set point
        break;
      case L1_POSITION:
        m_controller.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedFoward); // the first value is
                                                                                                // the set point
        break;
      case L2_POSITION:
        m_controller.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedFoward); // the first value is
                                                                                                // the set point
        break;
      case L3_POSITION:
        m_controller.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedFoward); // the first value is
                                                                                                // the set point
        break;
      case L4_POSITION:
        m_controller.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedFoward); // the first value is
                                                                                                // the set point
        break;
    }

  }
}
