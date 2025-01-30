// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; ///// idk if this needs to be here but i think this is for the thing we use to monitor robot happenings

public class Elevator extends SubsystemBase {

  private SparkMax elevatorMotor;
  private SparkMaxConfig elevatorMotorConfig;
  private SparkLimitSwitch reverseLimitSwitch;
  private RelativeEncoder elevatorEncoder;

  SparkClosedLoopController m_controller;

  private double feedFoward = 8;

 
  public enum ElevatorState { //Creates a new Elevator.
    HOME,
    L1_POSITION,
    L2_POSITION,
    L3_POSITION,
    L4_POSITION,
    CORAL_POSITION
  }

  double homeTarget = 0;
  double l1Target = 0;
  double l2Target = 10;
  double l3Target = 20;
  double l4Target = 30;
  double coral_positionTarget = 40;

  double targetPosition;


  private ElevatorState uppydowny = ElevatorState.HOME;

  public Elevator() {
    elevatorMotorConfig = new SparkMaxConfig();
    elevatorMotor = new SparkMax(1, MotorType.kBrushless);
    reverseLimitSwitch = elevatorMotor.getReverseLimitSwitch();
    elevatorEncoder = elevatorMotor.getEncoder();//TODO:Encoder should be configured (position conversion factor), based on cad, in order to use the feedback

    m_controller = elevatorMotor.getClosedLoopController();
    elevatorMotorConfig.idleMode(IdleMode.kBrake);
    elevatorMotorConfig.limitSwitch
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

    elevatorMotorConfig.softLimit
        .forwardSoftLimit(50)  // TODO: chanege limet value
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true);

    SparkMaxConfig config = new SparkMaxConfig();

    // Set Smart Motion and Smart Velocity parameters.
    elevatorMotorConfig.closedLoop.maxMotion
        .maxVelocity(5) // TODO find these desigherd values
        .maxAcceleration(5)
        .allowedClosedLoopError(5);  
    // Set PID gains
    config.closedLoop // pid loop to contoll elevator elevating rate
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
        targetPosition = homeTarget;                                                                            // the set point
        break;
      case L1_POSITION:
        targetPosition = l1Target;                                                                                  // the set point
        break;
      case L2_POSITION:
         targetPosition = l2Target;                                                                                                // the set point
        break;
      case L3_POSITION:
        targetPosition = l3Target;                                                                                     // the set point
        break;
      case L4_POSITION:
        targetPosition = l4Target;                                                                                    // the set point
        break;
      case CORAL_POSITION:
        targetPosition = coral_positionTarget;                                                                                    // the set point
        break;
    }
    m_controller.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedFoward);
  }
  public void L1(){
    uppydowny = ElevatorState.L1_POSITION;
  }
  public void L2(){
    uppydowny = ElevatorState.L2_POSITION;
  }
  public void L3(){
    uppydowny = ElevatorState.L3_POSITION;
  }
  public void L4(){
    uppydowny = ElevatorState.L4_POSITION;
  }
  public void HOME(){
    uppydowny = ElevatorState.HOME;
  }
  public void coral_position(){
    uppydowny = ElevatorState.CORAL_POSITION;
  }

  public Command getL1Command() {
    return new RunCommand(
      ()->{
        L1();
      })
      .finallyDo(()-> L1());
    }  
  public Command getL2Command() {
      return new RunCommand(
      ()->{
        L2();
      })
      .finallyDo(()-> L2());
    } 
  public Command getL3Command() {
      return new RunCommand(
    ()->{
      L3();
    })
      .finallyDo(()-> L3());
    } 
  public Command getL4Command() {
      return new RunCommand(
    ()->{
      L4();
    })
      .finallyDo(()-> L4());
    } 
  public Command getCoral_positoonCommand() {
      return new RunCommand(
    ()->{
      coral_position();
    })
      .finallyDo(()-> coral_position());
    } 

}

