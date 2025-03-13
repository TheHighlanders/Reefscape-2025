package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Naming;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Align;
import frc.robot.subsystems.Elevator.ElevatorState;
import java.util.EnumMap;

@Logged(importance = Importance.INFO, defaultNaming = Naming.USE_HUMAN_NAME)
public class Superstructure extends SubsystemBase {

  public enum SuperState {
    IDLE,
    ALIGNINGLEFT,
    ALIGNINGRIGHT,
    DRIVE_FIELD_RELATIVE,
    DRIVE_ROBOT_RELATIVE,
    PRE_L1,
    PRE_L2,
    PRE_L3,
    PRE_L4,
    SCORE_CORAL
  }

  public enum CoralLevel {
    L1,
    L2,
    L3,
    L4
  }

  private final Swerve swerve;
  private final Elevator elevator;
  private final CoralScorer coralScorer;

  @Logged(importance = Importance.CRITICAL, name = "Current State")
  private SuperState state = SuperState.IDLE;

  @Logged(name = "Previous State")
  private SuperState prevState = SuperState.IDLE;

  @Logged(importance = Importance.CRITICAL, name = "Target Coral Level")
  private CoralLevel targetCoralLevel = CoralLevel.L4; // Default to L4

  @Logged(importance = Importance.INFO, name = "Drive Mode Robot Relative")
  private boolean isRobotRelative = false;

  private final Trigger autoAlignReqRight;
  private final Trigger autoAlignReqLeft;

  private final Trigger scoreReq;

  private final Trigger levelUpReq;

  private final Trigger levelDownReq;

  private final Trigger elevatorReady;

  private final Command autoAlignCommandRight;
  private final Command autoAlignCommandLeft;

  private final Command vibrationCommand;

  private final EnumMap<SuperState, Trigger> stateTriggers = new EnumMap<>(SuperState.class);
  private final Timer stateTimer = new Timer();

  public Superstructure(
      Swerve swerve,
      Elevator elevator,
      CoralScorer coralScorer,
      Trigger autoAlignReqLeft,
      Trigger autoAlignReqRight,
      Trigger scoreReq,
      Trigger levelUpReq,
      Trigger levelDownReq,
      Command vibrationCommand) {

    this.swerve = swerve;
    this.elevator = elevator;
    this.coralScorer = coralScorer;
    this.autoAlignReqRight = autoAlignReqRight;
    this.autoAlignReqLeft = autoAlignReqLeft;
    this.scoreReq = scoreReq;
    this.levelUpReq = levelUpReq;
    this.levelDownReq = levelDownReq;
    this.vibrationCommand = vibrationCommand;

    stateTimer.start();

    // Create auto align command
    autoAlignCommandRight = new Align(swerve, () -> true, vibrationCommand);
    autoAlignCommandLeft = new Align(swerve, () -> false, vibrationCommand);

    // Initialize triggers for each state
    for (var superState : SuperState.values()) {
      stateTriggers.put(
          superState, new Trigger(() -> this.state == superState && DriverStation.isEnabled()));
    }

    elevatorReady = new Trigger(() -> elevator.isAtSetpoint(0.5));

    configureStateTransitionCommands();
  }

  @Logged(importance = Importance.INFO, name = "Is Pre-Score State")
  public boolean isPreScoreState() {
    return state == SuperState.PRE_L1
        || state == SuperState.PRE_L2
        || state == SuperState.PRE_L3
        || state == SuperState.PRE_L4;
  }

  @Logged(importance = Importance.DEBUG, name = "Is Robot Relative")
  public boolean isRobotRelative() {
    return isRobotRelative;
  }

  @Logged(importance = Importance.INFO, name = "Current Target Coral Level")
  public CoralLevel getTargetCoralLevel() {
    return targetCoralLevel;
  }

  @Logged(importance = Importance.DEBUG, name = "State Timer Value")
  public double getStateTimerValue() {
    return stateTimer.get();
  }

  private void configureStateTransitionCommands() {
    // Handle auto align request
    autoAlignReqRight.onTrue(
        Commands.runOnce(
            () -> {
              isRobotRelative = true;
              forceState(SuperState.ALIGNINGRIGHT);
            }));

    autoAlignReqLeft.onTrue(
        Commands.runOnce(
            () -> {
              isRobotRelative = true;
              forceState(SuperState.ALIGNINGLEFT);
            }));

    autoAlignReqRight
        .or(autoAlignReqLeft)
        .onFalse(
            Commands.runOnce(
                () -> {
                  isRobotRelative = false;
                  if (state == SuperState.ALIGNINGLEFT || state == SuperState.ALIGNINGRIGHT) {
                    forceState(getCorrespondingPreState());
                  }
                }));

    // Handle scoring request
    scoreReq
        .and(() -> isPreScoreState())
        .onTrue(
            Commands.runOnce(
                () -> {
                  forceState(SuperState.SCORE_CORAL);
                }));

    // Handle coral level changes from operator
    levelUpReq.onTrue(Commands.runOnce(this::incrementCoralLevel));

    levelDownReq.onTrue(Commands.runOnce(this::decrementCoralLevel));

    // ALIGNING state behavior
    stateTriggers
        .get(SuperState.ALIGNINGRIGHT)
        .whileTrue(autoAlignCommandRight)
        .whileTrue(elevator.setPosition(ElevatorState.HOME));

    stateTriggers
        .get(SuperState.ALIGNINGLEFT)
        .whileTrue(autoAlignCommandLeft)
        .whileTrue(elevator.setPosition(ElevatorState.HOME));

    // PRE_L1 state behavior
    stateTriggers.get(SuperState.PRE_L1).whileTrue(elevator.setPosition(ElevatorState.HOME));

    // PRE_L2 state behavior
    stateTriggers.get(SuperState.PRE_L2).whileTrue(elevator.setPosition(ElevatorState.L2_POSITION));

    // PRE_L3 state behavior
    stateTriggers.get(SuperState.PRE_L3).whileTrue(elevator.setPosition(ElevatorState.L3_POSITION));

    // PRE_L4 state behavior
    stateTriggers.get(SuperState.PRE_L4).whileTrue(elevator.setPosition(ElevatorState.L4_POSITION));

    scoreReq
        .and(elevatorReady)
        .onTrue(
            Commands.runOnce(
                () -> {
                  forceState(SuperState.SCORE_CORAL);
                }));

    // SCORE_CORAL state behavior
    stateTriggers.get(SuperState.SCORE_CORAL).whileTrue(coralScorer.slowDepositCMD());
  }

  private SuperState getCorrespondingPreState() {
    return switch (targetCoralLevel) {
      case L1 -> SuperState.PRE_L1;
      case L2 -> SuperState.PRE_L2;
      case L3 -> SuperState.PRE_L3;
      case L4 -> SuperState.PRE_L4;
      default -> SuperState.PRE_L4;
    };
  }

  private void incrementCoralLevel() {
    switch (targetCoralLevel) {
      case L1 -> targetCoralLevel = CoralLevel.L2;
      case L2 -> targetCoralLevel = CoralLevel.L3;
      case L3 -> targetCoralLevel = CoralLevel.L4;
      case L4 -> targetCoralLevel = CoralLevel.L4;
    }
  }

  private void decrementCoralLevel() {
    switch (targetCoralLevel) {
      case L1 -> targetCoralLevel = CoralLevel.L1;
      case L2 -> targetCoralLevel = CoralLevel.L1;
      case L3 -> targetCoralLevel = CoralLevel.L2;
      case L4 -> targetCoralLevel = CoralLevel.L3;
    }
  }

  private Command forceState(SuperState nextState) {
    return Commands.runOnce(
            () -> {
              System.out.println("Changing state to " + nextState);
              stateTimer.reset();
              stateTimer.start();
              this.prevState = this.state;
              this.state = nextState;
            })
        .ignoringDisable(true);
  }
}
