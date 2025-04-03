// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Swerve;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

/** Add your docs here. */
public class Autos {

  AutoFactory autoFactory;
  Swerve drive;
  Elevator elevator;
  CoralScorer coral;
  Trigger canAlign;
  Supplier<Command> alignToLeftCoral;
  Supplier<Command> alignToRightCoral;

  public Autos(
      Swerve drive,
      Elevator elevator,
      CoralScorer coral,
      Trigger canAlign,
      Supplier<Command> alignToLeftCoral,
      Supplier<Command> alignToRightCoral) {
    autoFactory =
        new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::resetOdometry, // A function that resets the current robot pose to the provided
            // Pose2d
            drive::followTraj, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            drive // The drive subsystem
            );

    this.drive = drive;
    this.elevator = elevator;
    this.coral = coral;
    this.canAlign = canAlign;
    this.alignToLeftCoral = alignToLeftCoral;
    this.alignToRightCoral = alignToRightCoral;
  }

  public Command score() {
    return elevator
        .elevatorAuto(ElevatorState.L4_POSITION)
        .withName("Elevator Up")
        .andThen(coral.slowDepositCMD().withTimeout(1).withName("Slow Deposit"))
        .withName("Deposit")
        .andThen(
            elevator
                .elevatorAuto(ElevatorState.HOME)
                .deadlineFor(coral.reverseCommand())
                .withName("Return Elevator Home"))
        .withName("Elevator Down");
  }

  public Command testTraj() {
    return autoFactory.trajectoryCmd("Test").withName("Test Trajectory");
  }

  public AutoRoutine testDriveTrajRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("testDrive");
    AutoTrajectory test = routine.trajectory("TestDrive");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                    updateTrajectoryPIDCMD().withName("Update PID"),
                    test.resetOdometry().withName("Reset Odometry"),
                    test.cmd().withName("Follow Test Drive Path"))
                .withName("Test Drive Sequence"));

    return routine;
  }

  public AutoRoutine testRotateTrajRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("testRotate");
    AutoTrajectory test = routine.trajectory("TestRotate");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                    updateTrajectoryPIDCMD().withName("Update PID"),
                    test.resetOdometry().withName("Reset Odometry"),
                    test.cmd().withName("Follow Test Rotate Path"))
                .withName("Test Rotate Sequence"));

    return routine;
  }

  public AutoRoutine testDriveRotateTrajRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("testDriveRotate");
    AutoTrajectory test = routine.trajectory("TestRotateAndDrive");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                    updateTrajectoryPIDCMD().withName("Update PID"),
                    test.resetOdometry().withName("Reset Odometry"),
                    test.cmd().withName("Follow Test Rotate And Drive Path"))
                .withName("Test Drive Rotate Sequence"));

    return routine;
  }

  public AutoRoutine LeftTwoPiece() {
    AutoRoutine routine = autoFactory.newRoutine("LeftTwoPiece");

    AutoTrajectory leftStartToleftFar =
        routine.trajectory("leftStart-leftFar"); // Drives from staging line to lineup start point
    AutoTrajectory leftFarToleftStation =
        routine.trajectory(
            "leftFar-leftStation"); // Drives from Far Left Segment Right Branch, to Station
    AutoTrajectory leftStationToleftClose =
        routine.trajectory("leftStation-leftClose"); // Drives from station to lineup start point

    routine
        .active()
        .onTrue(
            Commands.sequence(
                    leftStartToleftFar.resetOdometry().withName("Reset Odometry"),
                    leftStartToleftFar.cmd().withName("Drive to Left Far"))
                .withName("Initial Drive Sequence")); // Starts driving

    canAlign
        .and(leftStartToleftFar.active())
        .onTrue(
            alignToRightCoral
                .get()
                .withName("Align1")
                .andThen(score().withName("Score First Piece"))
                .andThen(leftFarToleftStation.cmd().withName("Drive to Station"))
                .withName("Drive to Station"));

    leftFarToleftStation
        .done()
        .onTrue(
            coral
                .biteCMD()
                .withName("Bite")
                .andThen(leftStationToleftClose.cmd().withName("Drive to Close Position"))
                .withName("Wait and Drive"));

    canAlign
        .and(leftStationToleftClose.active())
        .onTrue(
            alignToLeftCoral
                .get()
                .withName("Align2")
                // .alongWith(
                //     elevator.elevatorAuto(ElevatorState.L2_POSITION).withName("Elevator to L2"))
                .andThen(
                    score().withName("Score Second Piece").deadlineFor(elevator.slowDownElevator()))
                .withName("Align and Score"));

    return routine;
  }

  public AutoRoutine RightTwoPiece() {
    AutoRoutine routine = autoFactory.newRoutine("RightTwoPiece");

    AutoTrajectory rightStart_rightFar = routine.trajectory("rightStart-rightFar");
    AutoTrajectory rightFar_rightStation = routine.trajectory("rightFar-rightStation");
    AutoTrajectory rightStation_rightClose = routine.trajectory("rightStation-rightClose");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                    rightStart_rightFar.resetOdometry().withName("Reset Odometry"),
                    rightStart_rightFar.cmd().withName("Drive to Left Far"))
                .withName("Initial Drive Sequence")); // Starts driving

    canAlign
        .and(rightStart_rightFar.active())
        .onTrue(
            alignToRightCoral
                .get()
                .withName("Align1")
                .andThen(score().withName("Score First Piece"))
                .andThen(rightFar_rightStation.cmd().withName("Drive to Station"))
                .withName("Drive to Station"));

    rightFar_rightStation
        .done()
        .onTrue(
            coral
                .biteCMD()
                .withName("Bite")
                .andThen(rightStation_rightClose.cmd().withName("Drive to Close Position"))
                .withName("Wait and Drive"));

    canAlign
        .and(rightStation_rightClose.active())
        .onTrue(
            alignToLeftCoral
                .get()
                .withName("Align2")
                // .alongWith(
                //     elevator.elevatorAuto(ElevatorState.L2_POSITION).withName("Elevator to L2"))
                .andThen(
                    score().withName("Score Second Piece").deadlineFor(elevator.slowDownElevator()))
                .withName("Align and Score"));

    return routine;
  }

  public AutoRoutine CenterOnePiece() { // the one piece is real :>
    AutoRoutine routine = autoFactory.newRoutine("CenterOnePiece");

    AutoTrajectory centerStart_centerFar = routine.trajectory("centerStart-centerFar");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                    drive.disableVision(),
                    centerStart_centerFar.resetOdometry().withName("Reset Odometry"),
                    // drive.presetWheelsToTraj((SwerveSample)
                    // centerStart_centerFar.getRawTrajectory().getInitialSample(true).get()),
                    centerStart_centerFar.cmd().withName("Drive to Center Far"))
                .withName("Initial Drive Sequence"));

    centerStart_centerFar
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .withName("Elevator to L4")
                .andThen(Commands.waitSeconds(0.5).withName("Short Wait"))
                .andThen(coral.slowDepositCMD().withTimeout(3).withName("Deposit Coral"))
                .andThen(elevator.offsetElevator().withTimeout(0.2))
                .andThen(
                    Commands.deadline(
                        elevator.elevatorAuto(ElevatorState.HOME).withName("Elevator Home")))
                .withName("Score Sequence")
                .deadlineFor(elevator.slowDownElevator()));

    return routine;
  }

  public AutoRoutine CenterOnePieceAndLeftStation() { // the one piece is real :>
    AutoRoutine routine = autoFactory.newRoutine("CenterOnePieceAndLeftStation");

    AutoTrajectory centerStart_centerFar = routine.trajectory("centerStart-centerFar");
    AutoTrajectory centerStart_centerFar_Left_station =
        routine.trajectory("centerStart-centerFar-Left-station");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                    centerStart_centerFar.resetOdometry().withName("Reset Odometry"),
                    // drive.presetWheelsToTraj((SwerveSample)
                    // centerStart_centerFar.getRawTrajectory().getInitialSample(true).get()),

                    centerStart_centerFar.cmd().withName("Drive to Center Far"))
                .withName("Initial Drive Sequence"));

    centerStart_centerFar
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .withName("Elevator to L4")
                .andThen(Commands.waitSeconds(0.5).withName("Short Wait"))
                .andThen(coral.slowDepositCMD().withTimeout(3).withName("Deposit Coral"))
                .andThen(elevator.elevatorAuto(ElevatorState.HOME).withName("Elevator Home"))
                .andThen(centerStart_centerFar_Left_station.cmd().withName("Drive to Left Station"))
                .withName("Score and Move Sequence")); // move
    // elevator
    // then score
    // coral
    return routine;
  }

  public AutoRoutine CenterOnePieceAndRightStation() { // the one piece is real :>
    AutoRoutine routine = autoFactory.newRoutine("CenterOnePieceAndRightStation");

    AutoTrajectory centerStart_centerFar = routine.trajectory("centerStart-centerFar");
    AutoTrajectory centerStart_centerFar_right_station =
        routine.trajectory("centerStart-centerFar-right-station");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                    centerStart_centerFar.resetOdometry().withName("Reset Odometry"),
                    // drive.presetWheelsToTraj((SwerveSample)
                    // centerStart_centerFar.getRawTrajectory().getInitialSample(true).get()),

                    centerStart_centerFar.cmd().withName("Drive to Center Far"))
                .withName("Initial Drive Sequence"));

    centerStart_centerFar
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .withName("Elevator to L4")
                .andThen(Commands.waitSeconds(0.5).withName("Short Wait"))
                .andThen(coral.slowDepositCMD().withTimeout(3).withName("Deposit Coral"))
                .andThen(elevator.elevatorAuto(ElevatorState.HOME).withName("Elevator Home"))
                .andThen(
                    centerStart_centerFar_right_station.cmd().withName("Drive to Right Station"))
                .withName("Score and Move Sequence")); // move
    // elevator
    // then score
    // coral

    return routine;
  }

  public AutoRoutine CenterOnePieceAndDislodge() { // the one piece is real :>
    AutoRoutine routine = autoFactory.newRoutine("Center-dingus");

    AutoTrajectory CD1 = routine.trajectory("Center-dingus-one");
    AutoTrajectory CD2 = routine.trajectory("Center-dingus-two");
    AutoTrajectory CD3 = routine.trajectory("Center-dingus-three");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                    CD1.resetOdometry().withName("Reset Odometry"),
                    // drive.presetWheelsToTraj((SwerveSample)
                    // centerStart_centerFar.getRawTrajectory().getInitialSample(true).get()),
                    CD1.cmd().withName("Drive First Path"))
                .withName("Initial Drive Sequence"));

    CD1.done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .withName("Elevator to L4")
                .andThen(Commands.waitSeconds(0.5).withName("Short Wait"))
                .andThen(coral.slowDepositCMD().withTimeout(3).withName("Deposit Coral"))
                .andThen(CD2.cmd().withName("Drive Second Path"))
                .withName("Score and Move Sequence"));
    CD2.done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.ALGAELOW)
                .withName("Elevator to Algae Low")
                .andThen(
                    Commands.waitSeconds(2)
                        .withName("Wait for Algae")
                        .andThen(CD3.cmd().withName("Drive Third Path")))
                .withName("Algae Collection Sequence"));
    CD3.done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.HOME)
                .withName("Elevator Home")
                .andThen(Commands.waitSeconds(1).withName("Final Wait"))
                .withName("Return Home Sequence"));
    return routine;
  }

  public AutoRoutine ThreePieceLeft() { // the one piece is real :>
    AutoRoutine routine = autoFactory.newRoutine("ThreePieceLeft");

    AutoTrajectory centerStartTocenterFar = routine.trajectory("centerStart-centerFar");
    AutoTrajectory centerStarttoLeftStation =
        routine.trajectory("centerStart-centerFar-Left-station");
    AutoTrajectory LeftStationleftFar = routine.trajectory("leftStation-leftFar");
    AutoTrajectory leftFarToLeftStation = routine.trajectory("leftFar-leftStation");
    AutoTrajectory LeftStationToLeftClose = routine.trajectory("leftStation-leftClose");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                    centerStartTocenterFar.resetOdometry().withName("Reset Odometry"),
                    // drive.presetWheelsToTraj((SwerveSample)
                    // centerStart_centerFar.getRawTrajectory().getInitialSample(true).get()),
                    centerStartTocenterFar.cmd().withName("Drive to Center Far"))
                .withName("Initial Drive Sequence"));

    centerStartTocenterFar
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .withName("Elevator to L4")
                .andThen(Commands.waitSeconds(0.5).withName("Short Wait"))
                .andThen(
                    coral
                        .slowDepositCMD()
                        .withName("Deposit Coral")
                        .withTimeout(2.5)
                        .andThen(
                            elevator.elevatorAuto(ElevatorState.HOME).withName("Elevator Home"))
                        .andThen(centerStarttoLeftStation.cmd().withName("Drive to Left Station"))
                        .withName("Score and Move Sequence")));

    centerStarttoLeftStation
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.HOME)
                .withName("Ensure Elevator Home")
                .andThen(coral.intakeCMD().withTimeout(1).withName("Intake Coral"))
                .andThen(LeftStationleftFar.cmd().withName("Drive to Left Far"))
                .withName("Intake and Move Sequence"));

    LeftStationleftFar.done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .withName("Elevator to L4")
                .andThen(Commands.waitSeconds(0.5).withName("Short Wait"))
                .andThen(
                    coral
                        .slowDepositCMD()
                        .withName("Deposit Coral")
                        .withTimeout(2.5)
                        .andThen(
                            elevator.elevatorAuto(ElevatorState.HOME).withName("Elevator Home"))
                        .andThen(leftFarToLeftStation.cmd().withName("Return to Left Station"))
                        .withName("Second Score and Move Sequence")));

    leftFarToLeftStation
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.HOME)
                .withName("Ensure Elevator Home")
                .andThen(coral.intakeCMD().withTimeout(1).withName("Intake Coral"))
                .andThen(LeftStationleftFar.cmd().withName("Drive to Left Far Again"))
                .withName("Second Intake and Move Sequence"));

    LeftStationToLeftClose.done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .withName("Elevator to L4")
                .andThen(Commands.waitSeconds(0.5).withName("Short Wait"))
                .andThen(
                    coral
                        .slowDepositCMD()
                        .withName("Deposit Coral")
                        .withTimeout(2.5)
                        .andThen(
                            elevator.elevatorAuto(ElevatorState.HOME).withName("Elevator Home"))
                        .withName("Final Score Sequence")));

    return routine;
  }

  public AutoRoutine ThreePieceRight() { // the one piece is real :>
    AutoRoutine routine = autoFactory.newRoutine("ThreePieceLeft");

    AutoTrajectory centerStartTocenterFar = routine.trajectory("centerStart-centerFar");
    AutoTrajectory centerFarToRightStation =
        routine.trajectory("centerStart-centerFar-Right-station");
    AutoTrajectory rightStationToRightFar = routine.trajectory("rightStation-rightFar");
    AutoTrajectory rightFarToRightStation = routine.trajectory("rightFar-rightStation");
    AutoTrajectory rightStationToRightClose = routine.trajectory("rightStation-rightClose");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                    centerStartTocenterFar.resetOdometry().withName("Reset Odometry"),
                    // drive.presetWheelsToTraj((SwerveSample)
                    // centerStart_centerFar.getRawTrajectory().getInitialSample(true).get()),
                    centerStartTocenterFar.cmd().withName("Drive to Center Far"))
                .withName("Initial Drive Sequence"));

    centerStartTocenterFar
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .withName("Elevator to L4")
                .andThen(Commands.waitSeconds(0.5).withName("Short Wait"))
                .andThen(
                    coral
                        .slowDepositCMD()
                        .withName("Deposit Coral")
                        .withTimeout(2.5)
                        .andThen(
                            elevator.elevatorAuto(ElevatorState.HOME).withName("Elevator Home"))
                        .andThen(centerFarToRightStation.cmd().withName("Drive to Right Station"))
                        .withName("Score and Move Sequence")));

    centerFarToRightStation
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.HOME)
                .withName("Ensure Elevator Home")
                .andThen(coral.intakeCMD().withTimeout(1).withName("Intake Coral"))
                .andThen(rightStationToRightFar.cmd().withName("Drive to Right Far"))
                .withName("Intake and Move Sequence"));

    rightStationToRightFar
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .withName("Elevator to L4")
                .andThen(Commands.waitSeconds(0.5).withName("Short Wait"))
                .andThen(
                    coral
                        .slowDepositCMD()
                        .withName("Deposit Coral")
                        .withTimeout(2.5)
                        .andThen(
                            elevator
                                .elevatorAuto(ElevatorState.HOME)
                                .withName("Elevator Home")
                                .andThen(
                                    rightFarToRightStation
                                        .cmd()
                                        .withName("Return to Right Station"))
                                .withName("Return to Station Sequence")))
                .withName("Second Score Sequence"));

    rightFarToRightStation
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.HOME)
                .withName("Ensure Elevator Home")
                .andThen(coral.intakeCMD().withTimeout(1).withName("Intake Coral"))
                .andThen(rightStationToRightClose.cmd().withName("Drive to Right Close"))
                .withName("Second Intake and Move Sequence"));

    rightStationToRightClose
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .withName("Elevator to L4")
                .andThen(Commands.waitSeconds(0.5).withName("Short Wait"))
                .andThen(
                    coral
                        .slowDepositCMD()
                        .withName("Deposit Coral")
                        .withTimeout(2.5)
                        .andThen(
                            elevator.elevatorAuto(ElevatorState.HOME).withName("Elevator Home"))
                        .withName("Final Score Sequence")));

    return routine;
  }

  public Command updateTrajectoryPIDCMD() {
    return Commands.runOnce(drive::updateTrajectoryPID).withName("Update Trajectory PID");
  }

  public Command simpleDriveAuto() {
    return drive.driveForwardTimed(1, 4).withName("Simple Drive Forward");
  }

  public Command simple1Piece() { // the one piece is real
    return Commands.sequence(
            drive.driveForwardTimed(1.5, 1.5).withName("Drive to Scoring Position"),
            coral.slowDepositCMD().withTimeout(1).withName("Deposit Coral"),
            drive.driveForwardTimed(-0.5, 0.75).withName("Back Away"))
        .withName("Simple One Piece Auto");
  }

  public Command testSequenceCommand() {
    return Commands.sequence(drive.driveForwardTimed(1.5, 1.5).withName("Test Drive Forward"))
        .withName("Test Sequence");
  }

  private void emptyConsumer(double x) {}
}
