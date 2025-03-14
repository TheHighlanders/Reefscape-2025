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

  private static final double ELEVATOR_SETPOINT_TOLERANCE = 0.5;
  private static final double CORAL_DEPOSIT_TIMEOUT = 0.5;

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

    this.autoAlignCommandRight = new Align(swerve, () -> true, vibrationCommand);
    this.autoAlignCommandLeft = new Align(swerve, () -> false, vibrationCommand);

    this.elevator = elevator;
    this.coralScorer = coralScorer;
    this.autoAlignReqRight = autoAlignReqRight;
    this.autoAlignReqLeft = autoAlignReqLeft;
    this.scoreReq = scoreReq;
    this.levelUpReq = levelUpReq;
    this.levelDownReq = levelDownReq;

    stateTimer.start();

    // Initialize triggers for each state
    for (var superState : SuperState.values()) {
      stateTriggers.put(
          superState, new Trigger(() -> this.state == superState && DriverStation.isEnabled()));
    }

    elevatorReady = new Trigger(() -> elevator.isAtSetpoint(ELEVATOR_SETPOINT_TOLERANCE));

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
              transitionToState(SuperState.ALIGNINGRIGHT);
            }));

    autoAlignReqLeft.onTrue(
        Commands.runOnce(
            () -> {
              isRobotRelative = true;
              transitionToState(SuperState.ALIGNINGLEFT);
            }));

    autoAlignReqRight
        .or(autoAlignReqLeft)
        .onFalse(
            Commands.runOnce(
                () -> {
                  isRobotRelative = false;
                  if (state == SuperState.ALIGNINGLEFT || state == SuperState.ALIGNINGRIGHT) {
                    transitionToState(getCorrespondingPreState());
                  }
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
                  transitionToState(SuperState.SCORE_CORAL);
                }));

    // SCORE_CORAL state behavior
    stateTriggers
        .get(SuperState.SCORE_CORAL)
        .whileTrue(
            Commands.sequence(
                coralScorer.slowDepositCMD().withTimeout(CORAL_DEPOSIT_TIMEOUT),
                elevator.setPosition(ElevatorState.HOME), // Return elevator to home
                Commands.runOnce(() -> transitionToState(SuperState.IDLE)) // Return to IDLE state
                ));
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

  /** Set the target coral level directly. */
  public void setTargetCoralLevel(CoralLevel level) {
    this.targetCoralLevel = level;
  }

  /** Get the current state. Useful for checking state in autonomous routines. */
  public SuperState getState() {
    return state;
  }

  private void transitionToState(SuperState nextState) {
    DriverStation.reportWarning("Changing state to " + nextState + " from " + this.state, false);
    stateTimer.reset();
    stateTimer.start();
    this.prevState = this.state;
    this.state = nextState;
  }

  public Command forceState(SuperState nextState) {
    return Commands.runOnce(() -> transitionToState(nextState)).ignoringDisable(true);
  }
}
