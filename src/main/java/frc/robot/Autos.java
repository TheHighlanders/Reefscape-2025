// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.CoralScorer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class Autos {
  AutoFactory autoFactory;
  Swerve drive;
  Elevator elevator;
  CoralScorer coral;

  public Autos(Swerve drive, Elevator elevator, CoralScorer coral) {
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

    AutoTrajectory leftStartToleftFar = routine.trajectory("leftStart-leftFar");
    AutoTrajectory leftFarToleftStation = routine.trajectory("leftFar-leftStation");
    AutoTrajectory leftStationToleftClose = routine.trajectory("leftStation-leftClose");

    routine
        .active()
        .onTrue(Commands.sequence(leftStartToleftFar.resetOdometry(), leftStartToleftFar.cmd()));

    leftStartToleftFar
        .atTime("WAIT")
        .onTrue(Commands.waitSeconds(5));

    leftStartToleftFar
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .andThen(coral.depositCMD().withTimeout(.5))
                .andThen(elevator.elevatorAuto(ElevatorState.HOME))
                .andThen(leftFarToleftStation.cmd())); // move elevator then score coral

    leftFarToleftStation
        .done()
        .onTrue(Commands.waitSeconds(2).andThen(leftStationToleftClose.cmd()));


    leftStationToleftClose
        .atTime("WAIT")
        .onTrue(Commands.waitSeconds(5));

    leftStationToleftClose
        .done()
        .onTrue(
            elevator.elevatorAuto(ElevatorState.L4_POSITION)
            /* .andThen(coral.depositCMD().withTimeout(.5)) */ ); // move elevator then score coral

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
    
    // rightStart_rightFar
    //     .atTime("WAIT")
    //     .onTrue(Commands.waitSeconds(5));

    rightStart_rightFar
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.L4_POSITION)
                .andThen(coral.depositCMD().withTimeout(.5))
                .andThen(rightFar_rightStation.cmd())); // move elevator then score coral

    rightFar_rightStation
        .done()
        .onTrue(
            elevator
                .elevatorAuto(ElevatorState.HOME)
                .andThen(coral.intakeCMD().withTimeout(1))
                .andThen(rightStation_rightClose.cmd()));
   
    // rightStation_rightClose
    //     .atTime("WAIT")
    //     .onTrue(Commands.waitSeconds(5));

    rightStation_rightClose
        .done()
        .onTrue(
            elevator.elevatorAuto(ElevatorState.L4_POSITION)
            // .andThen(coral.depositCMD().withTimeout(.5))
            ); // move elevator then score coral

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
                .andThen(
                    elevator.elevatorAuto(ElevatorState.HOME))); // move elevator then score coral

    return routine;
  }
  public AutoRoutine CenterOnePieceAndLeftStation() { // the one piece is real :>
    AutoRoutine routine = autoFactory.newRoutine("CenterOnePieceAndLeftStation");

    AutoTrajectory centerStart_centerFar = routine.trajectory("centerStart-centerFar");
    AutoTrajectory centerStart_centerFar_Left_station = routine.trajectory("centerStart-centerFar-Left-station");
    
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
                .andThen(
                    elevator.elevatorAuto(ElevatorState.HOME))
                .andThen(centerStart_centerFar_Left_station.cmd())
                    ); // move elevator then score coral
    return routine;
  }

  public AutoRoutine CenterOnePieceAndRightStation() { // the one piece is real :>
    AutoRoutine routine = autoFactory.newRoutine("CenterOnePieceAndRightStation");

    AutoTrajectory centerStart_centerFar = routine.trajectory("centerStart-centerFar");
    AutoTrajectory centerStart_centerFar_right_station = routine.trajectory("centerStart-centerFar-right-station");
    
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
                .andThen(
                    elevator.elevatorAuto(ElevatorState.HOME))
                .andThen(centerStart_centerFar_right_station.cmd())
                    ); // move elevator then score coral

    return routine;
  }


  // public AutoRoutine practicepath() {
  //   AutoRoutine routine = autoFactory.newRoutine("LeftTwoPiece");

  //   AutoTrajectory practicepath = routine.trajectory("path-making-test");


  //   routine
  //       .active()
  //       .onTrue(Commands.sequence(practicepath.resetOdometry(), .cmd()));

  //   routine
  //       .active()
  //       .

  //   return routine;
  // }

  // public AutoRoutine CenterOnePieceToStation() { // the one piece is real :>
  //   AutoRoutine routine = autoFactory.newRoutine("CenterOnePieceToStation");

  //   AutoTrajectory centerStart_centerFar = routine.trajectory("centerStart-centerFar");
  //   AutoTrajectory centerStart_centerFar_left =
  // routine.trajectory("centerStart-centerFar-Left-station");
  //   routine
  //       .active()
  //       .onTrue(
  //           Commands.sequence(
  //               centerStart_centerFar.resetOdometry(),
  //               // drive.presetWheelsToTraj((SwerveSample)
  //               // centerStart_centerFar.getRawTrajectory().getInitialSample(true).get()),
  //               centerStart_centerFar.cmd()));

  //   centerStart_centerFar
  //       .done()
  //       .onTrue(
  //           elevator
  //               .elevatorAuto(ElevatorState.L4_POSITION)
  //               .andThen(Commands.waitSeconds(0.5))
  //               .andThen(coral.slowDepositCMD().withTimeout(3))
  //               .andThen(
  //                   elevator.elevatorAuto(ElevatorState.HOME)
  //                       .andThen(centerStart_centerFar_left.cmd())));
  //   return routine;
  // }

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
}
