// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.CoralLevel;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.Swerve;

/** Class to handle autonomous routines and trajectories. */
public class Autos {
  AutoFactory autoFactory;
  private final Swerve drive;
  private final Superstructure superstructure;

  public Autos(Swerve drive, Superstructure superstructure) {
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
    this.superstructure = superstructure;
  }

  /** Creates a command that aligns to the left coral and completes the entire scoring sequence. */
  private Command alignAndScoreLeftCoral() {
    return Commands.sequence(
        // Set target coral level if needed
        setCoralLevel(CoralLevel.L4),
        // Force the superstructure into ALIGNINGLEFT state
        Commands.runOnce(() -> superstructure.forceState(SuperState.ALIGNINGLEFT)),
        // Wait for the entire sequence to complete (state returns to IDLE)
        Commands.waitUntil(() -> superstructure.getState() == SuperState.IDLE)
            .withTimeout(6.0) // Reasonable timeout for the whole sequence
        );
  }

  /** Creates a command that aligns to the right coral and completes the entire scoring sequence. */
  private Command alignAndScoreRightCoral() {
    return Commands.sequence(
        // Set target coral level if needed
        setCoralLevel(CoralLevel.L4),
        // Force the superstructure into ALIGNINGRIGHT state
        Commands.runOnce(() -> superstructure.forceState(SuperState.ALIGNINGRIGHT)),
        // Wait for the entire sequence to complete (state returns to IDLE)
        Commands.waitUntil(() -> superstructure.getState() == SuperState.IDLE)
            .withTimeout(6.0) // Reasonable timeout for the whole sequence
        );
  }

  /** Command to set the target coral level */
  private Command setCoralLevel(CoralLevel level) {
    return Commands.runOnce(() -> superstructure.setTargetCoralLevel(level));
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
        .done()
        .onTrue(Commands.sequence(alignAndScoreLeftCoral(), leftFarToleftStation.cmd()));

    leftFarToleftStation
        .done()
        .onTrue(Commands.waitSeconds(2).andThen(leftStationToleftClose.cmd()));

    leftStationToleftClose.done().onTrue(alignAndScoreLeftCoral());

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
        .onTrue(Commands.sequence(alignAndScoreRightCoral(), rightFar_rightStation.cmd()));

    rightFar_rightStation
        .done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> superstructure.forceState(SuperState.IDLE)),
                Commands.waitSeconds(1.0),
                rightStation_rightClose.cmd()));

    rightStation_rightClose.done().onTrue(alignAndScoreRightCoral());

    return routine;
  }

  public AutoRoutine CenterOnePiece() {
    AutoRoutine routine = autoFactory.newRoutine("CenterOnePiece");

    AutoTrajectory centerStart_centerFar = routine.trajectory("centerStart-centerFar");

    routine
        .active()
        .onTrue(
            Commands.sequence(centerStart_centerFar.resetOdometry(), centerStart_centerFar.cmd()));

    centerStart_centerFar
        .done()
        .onTrue(
            Commands.sequence(
                // Direct scoring (no alignment needed)
                setCoralLevel(CoralLevel.L4),
                Commands.runOnce(() -> superstructure.forceState(SuperState.PRE_L4)),
                Commands.waitUntil(() -> superstructure.getState() == SuperState.IDLE)
                    .withTimeout(5.0)));

    return routine;
  }

  public AutoRoutine CenterOnePieceAndLeftStation() {
    AutoRoutine routine = autoFactory.newRoutine("CenterOnePieceAndLeftStation");

    AutoTrajectory centerStart_centerFar = routine.trajectory("centerStart-centerFar");
    AutoTrajectory centerStart_centerFar_Left_station =
        routine.trajectory("centerStart-centerFar-Left-station");

    routine
        .active()
        .onTrue(
            Commands.sequence(centerStart_centerFar.resetOdometry(), centerStart_centerFar.cmd()));

    centerStart_centerFar
        .done()
        .onTrue(
            Commands.sequence(
                // Direct scoring (no alignment needed)
                setCoralLevel(CoralLevel.L4),
                Commands.runOnce(() -> superstructure.forceState(SuperState.PRE_L4)),
                Commands.waitUntil(() -> superstructure.getState() == SuperState.IDLE)
                    .withTimeout(5.0),
                centerStart_centerFar_Left_station.cmd()));

    return routine;
  }

  public AutoRoutine CenterOnePieceAndRightStation() {
    AutoRoutine routine = autoFactory.newRoutine("CenterOnePieceAndRightStation");

    AutoTrajectory centerStart_centerFar = routine.trajectory("centerStart-centerFar");
    AutoTrajectory centerStart_centerFar_right_station =
        routine.trajectory("centerStart-centerFar-right-station");

    routine
        .active()
        .onTrue(
            Commands.sequence(centerStart_centerFar.resetOdometry(), centerStart_centerFar.cmd()));

    centerStart_centerFar
        .done()
        .onTrue(
            Commands.sequence(
                // Direct scoring (no alignment needed)
                setCoralLevel(CoralLevel.L4),
                Commands.runOnce(() -> superstructure.forceState(SuperState.PRE_L4)),
                Commands.waitUntil(() -> superstructure.getState() == SuperState.IDLE)
                    .withTimeout(5.0),
                centerStart_centerFar_right_station.cmd()));

    return routine;
  }

  public AutoRoutine CenterOnePieceAndDislodge() {
    AutoRoutine routine = autoFactory.newRoutine("Center-dingus");

    AutoTrajectory CD1 = routine.trajectory("Center-dingus-one");
    AutoTrajectory CD2 = routine.trajectory("Center-dingus-two");
    AutoTrajectory CD3 = routine.trajectory("Center-dingus-three");

    routine.active().onTrue(Commands.sequence(CD1.resetOdometry(), CD1.cmd()));

    CD1.done()
        .onTrue(
            Commands.sequence(
                // Direct scoring (no alignment needed)
                setCoralLevel(CoralLevel.L4),
                Commands.runOnce(() -> superstructure.forceState(SuperState.PRE_L4)),
                Commands.waitUntil(() -> superstructure.getState() == SuperState.IDLE)
                    .withTimeout(5.0),
                CD2.cmd()));

    CD2.done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> superstructure.forceState(SuperState.PRE_L1)),
                Commands.waitSeconds(2.0),
                CD3.cmd()));

    CD3.done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> superstructure.forceState(SuperState.IDLE)),
                Commands.waitSeconds(1.0)));

    return routine;
  }

  public AutoRoutine ThreePieceLeft() {
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
                centerStartTocenterFar.resetOdometry(), centerStartTocenterFar.cmd()));

    centerStartTocenterFar
        .done()
        .onTrue(
            Commands.sequence(
                // Direct scoring (no alignment needed)
                setCoralLevel(CoralLevel.L4),
                Commands.runOnce(() -> superstructure.forceState(SuperState.PRE_L4)),
                Commands.waitUntil(() -> superstructure.getState() == SuperState.IDLE)
                    .withTimeout(5.0),
                centerStarttoLeftStation.cmd()));

    centerStarttoLeftStation
        .done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> superstructure.forceState(SuperState.IDLE)),
                Commands.waitSeconds(1.0),
                LeftStationleftFar.cmd()));

    LeftStationleftFar.done()
        .onTrue(Commands.sequence(alignAndScoreLeftCoral(), leftFarToLeftStation.cmd()));

    leftFarToLeftStation
        .done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> superstructure.forceState(SuperState.IDLE)),
                Commands.waitSeconds(1.0),
                LeftStationToLeftClose.cmd()));

    LeftStationToLeftClose.done().onTrue(alignAndScoreLeftCoral());

    return routine;
  }

  public AutoRoutine ThreePieceRight() {
    AutoRoutine routine = autoFactory.newRoutine("ThreePieceRight");

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
                centerStartTocenterFar.resetOdometry(), centerStartTocenterFar.cmd()));

    centerStartTocenterFar
        .done()
        .onTrue(
            Commands.sequence(
                // Direct scoring (no alignment needed)
                setCoralLevel(CoralLevel.L4),
                Commands.runOnce(() -> superstructure.forceState(SuperState.PRE_L4)),
                Commands.waitUntil(() -> superstructure.getState() == SuperState.IDLE)
                    .withTimeout(5.0),
                centerFarToRightStation.cmd()));

    centerFarToRightStation
        .done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> superstructure.forceState(SuperState.IDLE)),
                Commands.waitSeconds(1.0),
                rightStationToRightFar.cmd()));

    rightStationToRightFar
        .done()
        .onTrue(Commands.sequence(alignAndScoreRightCoral(), rightFarToRightStation.cmd()));

    rightFarToRightStation
        .done()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> superstructure.forceState(SuperState.IDLE)),
                Commands.waitSeconds(1.0),
                rightStationToRightClose.cmd()));

    rightStationToRightClose.done().onTrue(alignAndScoreRightCoral());

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
        setCoralLevel(CoralLevel.L4),
        Commands.runOnce(() -> superstructure.forceState(SuperState.PRE_L4)),
        Commands.waitUntil(() -> superstructure.getState() == SuperState.IDLE).withTimeout(5.0),
        drive.driveForwardTimed(-0.5, 0.75));
  }

  public Command testSequenceCommand() {
    return Commands.sequence(drive.driveForwardTimed(1.5, 1.5));
  }
}
