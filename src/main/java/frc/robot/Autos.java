// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

/** Add your docs here. */
public class Autos {
  AutoFactory autoFactory;
  Swerve drive;
  Elevator elevator;
  CoralScorer coral;
  Supplier<Command> alignToLeftCoral;
  Supplier<Command> alignToRightCoral;

  public Autos(
      Swerve drive,
      Elevator elevator,
      CoralScorer coral,
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
    this.alignToLeftCoral = alignToLeftCoral;
    this.alignToRightCoral = alignToRightCoral;
  }

  public Command testTraj() {
    return autoFactory.trajectoryCmd("Test");
  }

  public AutoRoutine testDriveTrajRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("testDrive");
    AutoTrajectory test = routine.trajectory("TestDrive");

    routine
        .active()
        .onTrue(Commands.sequence(updateTrajectoryPIDCMD(), test.resetOdometry(), test.cmd()));

    return routine;
  }

  public AutoRoutine testRotateTrajRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("testRotate");
    AutoTrajectory test = routine.trajectory("TestRotate");

    routine
        .active()
        .onTrue(Commands.sequence(updateTrajectoryPIDCMD(), test.resetOdometry(), test.cmd()));

    return routine;
  }

  public AutoRoutine testDriveRotateTrajRoutine() {
    AutoRoutine routine = autoFactory.newRoutine("testDriveRotate");
    AutoTrajectory test = routine.trajectory("TestRotateAndDrive");

    routine
        .active()
        .onTrue(Commands.sequence(updateTrajectoryPIDCMD(), test.resetOdometry(), test.cmd()));

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
                leftStartToleftFar.resetOdometry(), leftStartToleftFar.cmd())); // Starts driving

    leftStartToleftFar
        .done()
        .onTrue(
            alignToRightCoral
                .get()
                .withTimeout(1.5)
                .alongWith(elevator.elevatorAuto(ElevatorState.L2_POSITION))
                .withName("Align1")
                .andThen(elevator.elevatorAuto(ElevatorState.L4_POSITION))
                .withName("Elevator Up")
                .andThen(coral.slowDepositCMD().withTimeout(1))
                .withName("Deposit")
                .andThen(elevator.elevatorAuto(ElevatorState.HOME))
                .withName("Elevator Down")
                .andThen(leftFarToleftStation.cmd())
                .withName("Drive to Station"));

    leftFarToleftStation
        .done()
        .onTrue(Commands.waitSeconds(0.25).andThen(leftStationToleftClose.cmd()));

    leftStationToleftClose
        .done()
        .onTrue(
            alignToLeftCoral
                .get()
                .withTimeout(1.5)
                .alongWith(elevator.elevatorAuto(ElevatorState.L2_POSITION))
                .withName("Align2")
                .andThen(elevator.elevatorAuto(ElevatorState.L4_POSITION))
                .withName("Elevator Up")
                .andThen(coral.slowDepositCMD().withTimeout(1))
                .withName("Deposit")
                .andThen(elevator.elevatorAuto(ElevatorState.HOME))
                .withName("Elevator Down"));

    return routine;
  }

  public AutoRoutine RightTwoPiece() {
    AutoRoutine routine = autoFactory.newRoutine("RightTwoPiece");

    AutoTrajectory rightStart_rightFar = routine.trajectory("rightStart-rightFar");
    AutoTrajectory rightFar_rightStation = routine.trajectory("rightFar-rightStation");
    AutoTrajectory rightStation_rightClose = routine.trajectory("rightStation-rightClose");

    routine
        .active()
        .onTrue(Commands.sequence(rightStart_rightFar.resetOdometry(), rightStart_rightFar.cmd()));

    rightStart_rightFar
        .done()
        .onTrue(
            alignToRightCoral
                .get()
                .withTimeout(1.25)
                .alongWith(elevator.elevatorAuto(ElevatorState.L2_POSITION))
                .andThen(elevator.elevatorAuto(ElevatorState.L4_POSITION))
                .andThen(coral.slowDepositCMD().withTimeout(1))
                .andThen(elevator.elevatorAuto(ElevatorState.HOME))
                .andThen(rightFar_rightStation.cmd())); // move

    rightFar_rightStation
        .done()
        .onTrue(Commands.waitSeconds(1).andThen(rightStation_rightClose.cmd()));

    rightStation_rightClose
        .done()
        .onTrue(
            alignToRightCoral
                .get()
                .withTimeout(1.25)
                .alongWith(elevator.elevatorAuto(ElevatorState.L2_POSITION))
                .andThen(elevator.elevatorAuto(ElevatorState.L4_POSITION))
                .andThen(coral.slowDepositCMD().withTimeout(1))
                .andThen(elevator.elevatorAuto(ElevatorState.HOME)));

    return routine;
  }

  public AutoRoutine CenterOnePiece() { // the one piece is real :>
    AutoRoutine routine = autoFactory.newRoutine("CenterOnePiece");

    AutoTrajectory centerStart_centerFar = routine.trajectory("centerStart-centerFar");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                centerStart_centerFar.resetOdometry(),
                // drive.presetWheelsToTraj((SwerveSample)
                // centerStart_centerFar.getRawTrajectory().getInitialSample(true).get()),
                centerStart_centerFar.cmd()));

    centerStart_centerFar
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(coral.slowDepositCMD().withTimeout(3))
                .andThen(elevator.elevatorAuto(ElevatorState.HOME)));

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
                centerStart_centerFar.resetOdometry(),
                // drive.presetWheelsToTraj((SwerveSample)
                // centerStart_centerFar.getRawTrajectory().getInitialSample(true).get()),

                centerStart_centerFar.cmd()));

    centerStart_centerFar
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(coral.slowDepositCMD().withTimeout(3))
                .andThen(elevator.elevatorAuto(ElevatorState.HOME))
                .andThen(centerStart_centerFar_Left_station.cmd())); // move
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
                centerStart_centerFar.resetOdometry(),
                // drive.presetWheelsToTraj((SwerveSample)
                // centerStart_centerFar.getRawTrajectory().getInitialSample(true).get()),

                centerStart_centerFar.cmd()));

    centerStart_centerFar
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(coral.slowDepositCMD().withTimeout(3))
                .andThen(elevator.elevatorAuto(ElevatorState.HOME))
                .andThen(centerStart_centerFar_right_station.cmd())); // move
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
                CD1.resetOdometry(),
                // drive.presetWheelsToTraj((SwerveSample)
                // centerStart_centerFar.getRawTrajectory().getInitialSample(true).get()),
                CD1.cmd()));

    CD1.done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(coral.slowDepositCMD().withTimeout(3))
                .andThen(CD2.cmd()));
    CD2.done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.ALGAELOW)
                .andThen(Commands.waitSeconds(2).andThen(CD3.cmd())));
    CD3.done().onTrue(elevator.elevatorAuto(ElevatorState.HOME).andThen(Commands.waitSeconds(1)));
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
                centerStartTocenterFar.resetOdometry(),
                // drive.presetWheelsToTraj((SwerveSample)
                // centerStart_centerFar.getRawTrajectory().getInitialSample(true).get()),
                centerStartTocenterFar.cmd()));

    centerStartTocenterFar
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(
                    coral
                        .slowDepositCMD()
                        .withTimeout(2.5)
                        .andThen(elevator.elevatorAuto(ElevatorState.HOME))
                        .andThen(centerStarttoLeftStation.cmd())));

    centerStarttoLeftStation
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.HOME)
                .andThen(coral.intakeCMD().withTimeout(1))
                .andThen(LeftStationleftFar.cmd()));

    LeftStationleftFar.done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(
                    coral
                        .slowDepositCMD()
                        .withTimeout(2.5)
                        .andThen(elevator.elevatorAuto(ElevatorState.HOME))
                        .andThen(leftFarToLeftStation.cmd())));

    leftFarToLeftStation
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.HOME)
                .andThen(coral.intakeCMD().withTimeout(1))
                .andThen(LeftStationleftFar.cmd()));

    LeftStationToLeftClose.done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(
                    coral
                        .slowDepositCMD()
                        .withTimeout(2.5)
                        .andThen(elevator.elevatorAuto(ElevatorState.HOME))));

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
                centerStartTocenterFar.resetOdometry(),
                // drive.presetWheelsToTraj((SwerveSample)
                // centerStart_centerFar.getRawTrajectory().getInitialSample(true).get()),
                centerStartTocenterFar.cmd()));

    centerStartTocenterFar
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(
                    coral
                        .slowDepositCMD()
                        .withTimeout(2.5)
                        .andThen(elevator.elevatorAuto(ElevatorState.HOME))
                        .andThen(centerFarToRightStation.cmd())));

    centerFarToRightStation
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.HOME)
                .andThen(coral.intakeCMD().withTimeout(1))
                .andThen(rightStationToRightFar.cmd()));

    rightStationToRightFar
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(
                    coral
                        .slowDepositCMD()
                        .withTimeout(2.5)
                        .andThen(
                            elevator
                                .elevatorAuto(ElevatorState.HOME)
                                .andThen(rightFarToRightStation.cmd()))));

    rightFarToRightStation
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.HOME)
                .andThen(coral.intakeCMD().withTimeout(1))
                .andThen(rightStationToRightClose.cmd()));

    rightStationToRightClose
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(
                    coral
                        .slowDepositCMD()
                        .withTimeout(2.5)
                        .andThen(elevator.elevatorAuto(ElevatorState.HOME))));

    return routine;
  }

  public Command updateTrajectoryPIDCMD() {
    return Commands.runOnce(drive::updateTrajectoryPID);
  }

  public Command simpleDriveAuto() {
    return drive.driveForwardTimed(1, 4);
  }

  public Command simple1Piece() { // the one piece is real
    return Commands.sequence(
        drive.driveForwardTimed(1.5, 1.5),
        coral.slowDepositCMD().withTimeout(1),
        drive.driveForwardTimed(-0.5, 0.75));
  }

  public Command testSequenceCommand() {
    return Commands.sequence(drive.driveForwardTimed(1.5, 1.5));
  }

  private void emptyConsumer(double x) {}
}
