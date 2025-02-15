// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.TimedRobot;
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

  Thread m_visionThread;

    m_visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(640, 480);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                // Put a rectangle on the image
                Imgproc.rectangle(
                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }
}
  }
}
