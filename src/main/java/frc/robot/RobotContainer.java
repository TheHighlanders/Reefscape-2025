// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.Align;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.utils.CommandXboxControllerSubsystem;

import choreo.auto.AutoChooser;

public class RobotContainer {

  public final CommandXboxControllerSubsystem driver = new CommandXboxControllerSubsystem(0);
  public final CommandXboxControllerSubsystem operator = new CommandXboxControllerSubsystem(1);

  CoralScorer coralScorer = new CoralScorer();
  Climber climber = new Climber();

  @Logged(name = "Elevator")
  Elevator elevator = new Elevator();

  Vision cameras = new Vision();

  @Logged(name = "Swerve")
  Swerve drive = new Swerve(cameras, elevator::getElevatorPosition);

  public Trigger canAlign = new Trigger(() -> Align.canAlign(drive, cameras));

  LEDs leds = new LEDs(canAlign);

  Autos autos =
      new Autos(
          drive, elevator, coralScorer, canAlign, this::alignToLeftCoral, this::alignToRightCoral);
  AutoChooser chooser;

  @Logged(name = "lastAlignedPose")
  private Pose2d lastAlignedPose = new Pose2d();

  @Logged(name = "lastAlignSide")
  private boolean lastAlignSide = true; // RIght -> true

  static final double ALIGN_THRESH = 1.0; // Meters till invalid realignment

  @Logged(name = "nextScoreHeight")
  ElevatorState nextScoreHeight = ElevatorState.L4_POSITION;

  @Logged(name = "Align")
  Command alignLogging =
      new Align(drive, cameras, canAlign, createDirectionalRumbleCallback(), leds);

  public RobotContainer() {

    chooser = new AutoChooser();

    configureBindings();
    configureAutonomous();

    drive.setDefaultCommand(
        drive
            .driveCMD(driver::getLeftX, driver::getLeftY, driver::getRightX)
            .withName("Default Drive Command"));

    cameraSetUp();
  }

  private void configureBindings() {
    Trigger manual = operator.rightTrigger();
    // Controls Spreadsheet \/
    // https://docs.google.com/spreadsheets/d/1bb3pvQep2hsePMl8YiyaagtdjtIkL8Z2Oqf_t2ND-68/edit?gid=0#gid=0
    operator
        .a()
        .and(manual.negate())
        .onTrue(elevator.setNextElevatorHeight(ElevatorState.HOME).withName("Set Elevator Home"));
    operator
        .x()
        .and(manual.negate())
        .onTrue(
            elevator.setNextElevatorHeight(ElevatorState.L2_POSITION).withName("Set Elevator L2"));
    operator
        .y()
        .and(manual.negate())
        .onTrue(
            elevator.setNextElevatorHeight(ElevatorState.L3_POSITION).withName("Set Elevator L3"));
    operator
        .b()
        .and(manual.negate())
        .onTrue(
            elevator.setNextElevatorHeight(ElevatorState.L4_POSITION).withName("Set Elevator L4"));

    operator
        .a()
        .and(manual)
        .onTrue(elevator.setPosition(ElevatorState.HOME).withName("Set Elevator Home"));
    operator
        .x()
        .and(manual)
        .onTrue(elevator.setPosition(ElevatorState.L2_POSITION).withName("Set Elevator L2"));
    operator
        .y()
        .and(manual)
        .onTrue(elevator.setPosition(ElevatorState.L3_POSITION).withName("Set Elevator L3"));
    operator
        .b()
        .and(manual)
        .onTrue(elevator.setPosition(ElevatorState.L4_POSITION).withName("Set Elevator L4"));

    operator.leftBumper().onTrue(elevator.algaeCMD(operator::getRightY).withName("Algae Control"));
    operator.leftTrigger(0.5).whileTrue(elevator.offsetElevator().withName("Offset Elevator"));

    operator.leftStick().and(manual.negate()).onTrue(coralScorer.biteCMD().withName("Bite Coral"));
    operator
        .leftStick()
        .and(manual)
        .whileTrue(coralScorer.manualIntakeCMD().withName("Manual Intake"));

    driver.start().whileTrue(elevator.zeroElevator().withName("Zero Elevator"));
    driver.povRight().onTrue(drive.resetGyro().withName("Reset Gyro"));

    driver.a().whileTrue(coralScorer.reverseCommand().withName("Reverse Coral"));

    driver.x().onTrue(drive.pointWheelsInXPattern().withName("X Pattern Wheels"));

    driver.b().whileTrue(drive.driveOrbit(driver::getLeftX, driver::getLeftY));

    driver.leftTrigger().onTrue(drive.enableSlowMode().withName("Enable Slow Mode"));
    driver.leftTrigger().onFalse(drive.disableSlowMode().withName("Disable Slow Mode"));

    driver.rightBumper().whileTrue(alignToRightCoral());
    driver.rightBumper().onTrue(Commands.runOnce(() -> this.lastAlignSide = true));

    driver
        .rightBumper()
        .onFalse(
            elevator
                .setPosition(ElevatorState.HOME)
                .alongWith(drive.disableSlowMode().withName("Disable Slow Mode")));

    driver.leftBumper().whileTrue(alignToLeftCoral());
    driver.leftBumper().onTrue(Commands.runOnce(() -> this.lastAlignSide = false));

    driver.rightTrigger().whileTrue(selectScoreRoutine());
    driver.rightTrigger().onFalse(removeAlgaeAndSlowMode());

    driver
        .leftBumper()
        .onFalse(
            elevator
                .setPosition(ElevatorState.HOME)
                .alongWith(drive.disableSlowMode().withName("Disable Slow Mode")));

    operator
        .povDown()
        .or(operator.povDownLeft())
        .or(operator.povDownRight())
        .whileTrue(climber.createClimbOutCommand().withName("Climb Out"));

    operator
        .povUp()
        .or(operator.povUpLeft())
        .or(operator.povUpRight())
        .whileTrue(climber.createClimbInCommand().withName("Climb In"));
    operator.povRight().onTrue(climber.holdClimbPosition().withName("Hold Climb Position"));

    operator
        .rightBumper()
        .and(manual)
        .onTrue(Commands.runOnce(coralScorer::effectorForward).withName("Quick Deposit"));

    operator
        .start()
        .onTrue(
            Commands.runOnce(() -> drive.resetOdometry(new Pose2d()))
                .ignoringDisable(true)
                .withName("Reset Odometry"));

    operator.back().and(manual).onTrue(Commands.runOnce(() -> cameraSetUp()));

    operator.start().onTrue(drive.resetPoseToVision());

    operator
        .rightStick()
        .whileTrue(
            elevator
                .trimCMD(operator::getRightY)
                .withName("Trim Elevator")
                .andThen(
                    elevator.setPosition(ElevatorState.CURRENT).withName("Set Current Position"))
                .withName("Trim and Set Elevator"));
  }

  private void configureAutonomous() {
    chooser.addRoutine("Left 2 Piece", autos::LeftTwoPiece);
    chooser.addRoutine("Right 2 Piece", autos::RightTwoPiece);
    chooser.addRoutine("Center 1 Piece", autos::CenterOnePiece);
    if (Constants.devMode) {
      chooser.addCmd("SYSID", drive::sysId);
      chooser.addCmd(
          "FORWARD",
          () ->
              Commands.sequence(
                      drive.enableSlowMode().withName("Enable Slow Mode"),
                      drive
                          .driveCMD(() -> 1, () -> 0, () -> 0)
                          .withTimeout(1)
                          .withName("Drive Forward"),
                      drive.disableSlowMode().withName("Disable Slow Mode"))
                  .withName("Forward Test Sequence"));
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
    alignLogging =
        new Align(drive, cameras, () -> true, createDirectionalRumbleCallback(), leds)
            .withName("Align to Right Coral");
    return alignLogging;
  }

  public Command scoreL1() {
    return Commands.parallel(
            coralScorer.slowDepositCMD(),
            drive.driveRobotRelativeCMD(() -> 0, () -> getL1Direction(), () -> 0))
        .until(coralScorer::doesNotHaveGamePiece);
  }

  public double getL1Direction() {
    return lastAlignSide ? 0.5 : 0.5;
  }

  public Command alignToLeftCoral() {
    alignLogging =
        new Align(drive, cameras, () -> false, createDirectionalRumbleCallback(), leds)
            .withName("Align to Left Coral");
    return alignLogging;
  }

  private BiConsumer<Double, Boolean> createDirectionalRumbleCallback() {
    return (xError, rumbleBoth) -> {
      DoubleSupplier leftIntensity;
      DoubleSupplier rightIntensity;
      // Max rumble at 5 cm
      double errorMagnitude = Math.min(1.0, Math.abs(xError) / 0.05);

      // If rumble both is true then xError is just the value of the rumble
      if (rumbleBoth) {
        rightIntensity = () -> xError;
        leftIntensity = () -> xError;
      } else {
        if (xError > 0) {
          rightIntensity = () -> errorMagnitude;
          leftIntensity = () -> 0;
        } else if (xError <= 0) {
          rightIntensity = () -> 0;
          leftIntensity = () -> errorMagnitude;
        } else {
          DriverStation.reportError("Driver rumble may be null", true);
          rightIntensity = () -> errorMagnitude;
          leftIntensity = () -> errorMagnitude;
        }
      }

      driver
          .rumbleCmd(leftIntensity, rightIntensity)
          .withTimeout(0.5)
          .withName("Rumble Command")
          .schedule();
    };
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

  public Command removeAlgae(ElevatorState height) {
    return Commands.deadline(elevator.elevatorAuto(height), coralScorer.reverseCommand());
  }

  public Command removeAlgaeLow() {
    return removeAlgae(ElevatorState.ALGAELOW);
  }

  public Command removeAlgaeHigh() {
    return removeAlgae(ElevatorState.ALGAEHIGH);
  }

  public Command selectScoreRoutine() {
    SmartDashboard.putBoolean("L1 last align side", lastAlignSide);
    return Commands.either(scoreL1(), autoScore(), elevator::nextHeightIsHome);
  }

  private Command autoScore() {
    return Commands.sequence(
        elevator.runToNextHeight(), coralScorer.depositCMD(elevator.getSetpoint()));
  }

  public Trigger isTryingToDrive() {
    return driver
        .axisMagnitudeGreaterThan(0, 0.05)
        .or(driver.axisMagnitudeGreaterThan(1, 0.05))
        .or(driver.axisMagnitudeGreaterThan(4, 0.05));
  }

  public Command removeAlgaeAndSlowMode() {
    return removeAlgae(ElevatorState.HOME)
        .alongWith(drive.disableSlowMode().withName("Disable Slow Mode"));
  }
}
