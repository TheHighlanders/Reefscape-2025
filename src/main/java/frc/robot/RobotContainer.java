// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Swerve;
import java.util.HashMap;
import java.util.Map;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;


public class RobotContainer {

  private final Map<String, Subsystem> subsystems = new HashMap<>();
  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);

  CoralScorer coralScorer = new CoralScorer();
  Climber climber = new Climber();
  Elevator elevator = new Elevator();
  Swerve drive = new Swerve(elevator::getElevatorPosition);
  Autos autos = new Autos(drive);

  AutoChooser chooser;

  public RobotContainer() {
    chooser = new AutoChooser();

    subsystems.put("drive", drive);
    subsystems.put("CoralScorer", coralScorer);
    subsystems.put("climber", climber);
    subsystems.put("elevator", elevator);

    configureBindings();
    configureAutonomous();

    // drive.setDefaultCommand(drive.driveCMD(driver::getLeftX, driver::getLeftY,
    // driver::getRightX));
  }

  private void configureBindings() {
    // driver.start().onTrue(drive.resetGyro());

    driver.x().onTrue(elevator.setPosition(ElevatorState.L3_POSITION));
    driver.y().onTrue(elevator.setPosition(ElevatorState.L2_POSITION));
    driver.a().onTrue(elevator.setPosition(ElevatorState.HOME));
    // driver
    //     .rightTrigger(0.5)
    //
    // .onTrue(elevator.setPosition(ElevatorState.L1_POSITION).alongWith(CoralScorer.intakeCMD()));

    // driver.leftTrigger(0.5).onTrue(CoralScorer.depositCMD());
    // driver.leftTrigger().whileTrue(drive.slowMode());

    // operator.y().whileTrue(climber.createClimbOutCommand());
    // operator.a().whileTrue(climber.createClimbInCommand());
  }

  private void configureAutonomous() {
    chooser.addRoutine("Test Routine", autos::testTrajRoutine);
    chooser.addCmd("SYSID", drive::sysId);

    SmartDashboard.putData("AutoChooser", chooser);
  }

  public Command getAutonomousCommand() {
    return chooser.selectedCommand();
  }

  private void cameraSetUp(){
    
    // Creates UsbCamera and MjpegServer [1] and connects them
    UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
    mjpegServer1.setSource(usbCamera);
    // Creates the CvSink and connects it to the UsbCamera
    CvSink cvSink = new CvSink("opencv_USB Camera 0");
    cvSink.setSource(usbCamera);
    // Creates the CvSource and MjpegServer [2] and connects them
    CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
    mjpegServer2.setSource(outputStream);

  }
}
