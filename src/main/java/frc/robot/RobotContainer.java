// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Align;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.utils.CommandXboxControllerSubsystem;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class RobotContainer {

  public final CommandXboxControllerSubsystem driver = new CommandXboxControllerSubsystem(0);
  public final CommandXboxControllerSubsystem operator = new CommandXboxControllerSubsystem(1);

  CoralScorer coralScorer = new CoralScorer();
  Climber climber = new Climber();

  @Logged(name = "Elevator")
  Elevator elevator = new Elevator();

  Vision[] cameras = {new Vision()};

  @Logged(name = "Swerve")
  Swerve drive = new Swerve(cameras, elevator::getElevatorPosition);

  Autos autos =
      new Autos(drive, elevator, coralScorer, this::alignToLeftCoral, this::alignToRightCoral);
  AutoChooser chooser;

  public RobotContainer() {
    chooser = new AutoChooser();

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
    operator.leftBumper().onTrue(elevator.algaeCMD(operator::getRightY));
    operator.leftTrigger(0.5).whileTrue(elevator.offsetElevator());

    // operator.leftBumper().whileTrue(elevator.offsetElevator());
    operator.leftStick().whileTrue(coralScorer.manualIntakeCMD());

    driver.start().whileTrue(elevator.zeroElevator());
    driver.povRight().onTrue(drive.resetGyro());

    driver.rightTrigger(0.5).whileTrue(coralScorer.depositCMD());
    driver.a().whileTrue(coralScorer.reverseCommand());

    driver.x().onTrue(drive.pointWheelsInXPattern());

    driver.leftTrigger().onTrue(drive.enableSlowMode());
    driver.leftTrigger().onFalse(drive.disableSlowMode());

    driver.leftBumper().whileTrue(alignToLeftCoral());
    driver.rightBumper().whileTrue(alignToRightCoral());

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

    // operator.start().toggleOnTrue(drive.pointWheelsForward());
    // operator.back().whileTrue(drive.pidTuningJogAngle());
    operator.rightBumper().onTrue(coralScorer.depositCMD().withTimeout(0.1));
    // operator.rightBumper().whileTrue(coralScorer.depositCMD());

    operator
        .start()
        .onTrue(Commands.runOnce(() -> drive.resetOdometry(new Pose2d())).ignoringDisable(true));

    operator
        .rightStick()
        .whileTrue(
            elevator
                .trimCMD(operator::getRightY)
                .andThen(elevator.setPosition(ElevatorState.CURRENT)));

    // operator.povUp().whileTrue(elevator.jogElevator(2));
    // operator.povDown().whileTrue(elevator.jogElevator(-2));
  }

  private void configureAutonomous() {
    chooser.addRoutine("Left 2 Piece", autos::LeftTwoPiece);
    chooser.addRoutine("Right 2 Piece", autos::RightTwoPiece);
    chooser.addRoutine("Center 1 Piece", autos::CenterOnePiece);
    chooser.addCmd("TimeBased 1 piece", autos::simple1Piece);
    chooser.addRoutine("Center 1 & Left", autos::CenterOnePieceAndLeftStation);
    chooser.addRoutine("Center 1 & Right", autos::CenterOnePieceAndRightStation);
    chooser.addRoutine("Center 1 & dingus", autos::CenterOnePieceAndDislodge);
    chooser.addRoutine("Three piece left", autos::ThreePieceLeft);
    chooser.addRoutine("Three piece right", autos::ThreePieceRight);
    if (Constants.devMode) {
      chooser.addCmd("SYSID", drive::sysId);
      chooser.addCmd(
          "FORWARD",
          () ->
              Commands.sequence(
                  drive.enableSlowMode(),
                  drive.driveCMD(() -> 1, () -> 0, () -> 0).withTimeout(1),
                  drive.disableSlowMode()));
      chooser.addRoutine("Test Drive Routine", autos::testDriveTrajRoutine);
      chooser.addRoutine("Test Rotate Routine", autos::testRotateTrajRoutine);
      chooser.addRoutine("Test Drive & Rotate Routine", autos::testDriveRotateTrajRoutine);
    }

    SmartDashboard.putData("AutoChooser", chooser);
  }

  public Command getAutonomousCommand() {
    return chooser.selectedCommand();
  }

  public Command alignToRightCoral() {
    return new Align(drive, cameras[0], () -> true, driver.rumbleCmd(0.5, 0.5).withTimeout(0.5));
  }

  public Command alignToLeftCoral() {
    return new Align(drive, cameras[0], () -> false, driver.rumbleCmd(0.5, 0.5).withTimeout(0.5));
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

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();

              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("DriverReefCam", 320, 240);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();
              while (!Thread.interrupted()) {
                if (cvSink.grabFrame(mat) == 0) {
                  outputStream.notifyError(cvSink.getError());
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
