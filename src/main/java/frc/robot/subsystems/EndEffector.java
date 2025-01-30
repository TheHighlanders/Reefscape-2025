// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// in this there will be a motor, a beam brake


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Motor;




public class EndEffector extends SubsystemBase {
  /** Creates a new EndEffector. */
  CANSparkMax motor;

  private boolean gamePiece;
  private DigitalInput beamBreak;


  public EndEffector() {
    //motor = new CANSparkMax();
    
    beamBreak = new DigitalInput(Constants.Intake.kIntakeBeamBreakDIOPin);
    gamePiece = hasGamePiece();

    motor = new CANSparkMaxCurrent(Constants.EndEffector.ENDEFFECTOR, MotorType.kBrushed);

    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    motor.setSpikeCurrentLimit(
        Constants.EndEffector.motorCurrentLimit.kLimitToAmps,
        Constants.EndEffector.motorCurrentLimit.kMaxSpikeTime,
        Constants.EndEffector.motorCurrentLimit.kMaxSpikeAmps,
        Constants.EndEffector.motorCurrentLimit.kSmartLimit
    );
}
  }

   @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Game Piece", hasGamePiece());
    }

    public boolean hasGamePiece() {
        return !beamBreak.get();
    }

    public void intakeStop() {
        DriverStation.reportWarning("Intake stop", false);
        motor.set(0);
    }

    public void intakeForward() {
        DriverStation.reportWarning("Intake forward", false);
        motor.set(1.0);
    }

    public void intakeReverse() {
        DriverStation.reportWarning("Intake reverse", false);
        motor.set(-1.0);
    }

}
