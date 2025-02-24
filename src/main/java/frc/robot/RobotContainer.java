// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
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
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class RobotContainer {

  private final Map<String, Subsystem> subsystems = new HashMap<>();
  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);

  CoralScorer coralScorer = new CoralScorer();
  Climber climber = new Climber();
  Elevator elevator = new Elevator();
  Swerve drive = new Swerve(elevator::getElevatorPosition);
  Autos autos = new Autos(drive, elevator, coralScorer);

  AutoChooser chooser;

  public RobotContainer() {
    chooser = new AutoChooser();

    subsystems.put("drive", drive);
    subsystems.put("CoralScorer", coralScorer);
    subsystems.put("climber", climber);
    subsystems.put("elevator", elevator);

    configureBindings();
    configureAutonomous();

    drive.setDefaultCommand(drive.driveCMD(driver::getLeftX, driver::getLeftY, driver::getRightX));

    cameraSetUp();
  }

  private void configureBindings() {
    // Controls Spreadsheet \/
    // https://docs.google.com/spreadsheets/d/1bb3pvQep2hsePMl8YiyaagtdjtIkL8Z2Oqf_t2ND-68/edit?gid=0#gid=0
    operator.a().onTrue(elevator.setPosition(ElevatorState.HOME));
    operator.x().onTrue(elevator.setPosition(ElevatorState.L2_POSITION));
    operator.y().onTrue(elevator.setPosition(ElevatorState.L3_POSITION));
    operator.b().onTrue(elevator.setPosition(ElevatorState.L4_POSITION));
    operator.leftBumper().whileTrue(elevator.offsetElevator());

    driver.start().whileTrue(elevator.zeroElevator());
    driver.povRight().onTrue(drive.resetGyro());

    driver.rightTrigger(0.5).whileTrue(coralScorer.depositCMD());
    driver.a().whileTrue(coralScorer.reverseCommand());
    driver.rightBumper().whileTrue(coralScorer.manualIntakeCMD());

    driver.leftTrigger().onTrue(drive.enableSlowMode());
    driver.leftTrigger().onFalse(drive.disableSlowMode());

    operator
        .povDown()
        .or(operator.povDownLeft())
        .or(operator.povDownRight())
        .whileTrue(climber.createClimbOutCommand());

    operator
        .povUp()
        .or(operator.povUpLeft())
        .or(operator.povUpRight())
        .whileTrue(climber.createClimbInCommand());
    operator.povRight().onTrue(climber.holdClimbPosition());

    operator.start().toggleOnTrue(drive.pointWheelsForward());
    operator.back().whileTrue(drive.pidTuningJogAngle());
    operator.rightBumper().whileTrue(coralScorer.depositCMD());

    // operator.povUp().whileTrue(elevator.jogElevator(2));
    // operator.povDown().whileTrue(elevator.jogElevator(-2));
  }

  private void configureAutonomous() {
    chooser.addRoutine("Test Drive Routine", autos::testDriveTrajRoutine);
    chooser.addRoutine("Test Rotate Routine", autos::testRotateTrajRoutine);
    chooser.addRoutine("Test Drive & Rotate Routine", autos::testDriveRotateTrajRoutine);
    chooser.addCmd("SYSID", drive::sysId);
    chooser.addCmd("FORWARD", () -> drive.pidTuningJogDrive());

    SmartDashboard.putData("AutoChooser", chooser);
  }

  public Command findClimberZero() {
    return climber.findZeroPosition();
  }

  public Command getAutonomousCommand() {
    return chooser.selectedCommand();
  }

  private void cameraSetUp() {

    Thread m_visionThread;

    m_visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(320, 240);
              camera.setPixelFormat(PixelFormat.kMJPEG);
            });

    m_visionThread =
        new Thread(
            () -> {

              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();

              // Set the resolution
              camera.setResolution(320, 240);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();

              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("DriverReefCam", 320, 240);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();
              while (!Thread.interrupted()) {
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }

                // Put a rectangle on the image
                Imgproc.rectangle(
                    mat, new Point(160, 240), new Point(160, 0), new Scalar(255, 0, 0), 5);
                // Give the output stream a new image to display

                outputStream.putFrame(mat);
              }
            });

    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }
}
