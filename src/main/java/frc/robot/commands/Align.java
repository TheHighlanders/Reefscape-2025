package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import java.util.function.BooleanSupplier;

@Logged
public class Align extends Command {

  /** Constants for the Reef Coral alignment */
  private static final class AlignConstants {
    // Distance from robot center to front bumper (meters)
    static final double robotCenterToFrontDistance = -0.39;

    // Lateral offset from tag to coral (meters)
    static final double coralLeftOffset = -0.165; // Left coral Y offset (negative = left)
    static final double coralRightOffset = 0.165; // Right coral Y offset (positive = right)

    static final double ejectOffset = 0.27;

    // Position tolerance (meters)
    static final double positionTolerance = 0.05;

    // Velocity tolerance (meters/s)
    static final double velocityTolerance = 0.1;

    // Rotation tolerance (radians)
    static final double rotationTolerance = 0.03;

    // Rotation velocity tolerance (rad/s)
    static final double rotationVelocityTolerance = 0.05;

    // Maximum approach speed (m/s)
    static final double maxApproachSpeed = 2.5;
    static final double maxApproachAccel = 5;

    // Maximum rotation speed (rad/s)
    static final double maxRotationSpeed = 5;
    static final double maxRotationAccel = 5;

    static final double translateP = 5;
    static final double translateI = 0.1;
    static final double translateD = 0;

    static final double rotateP = 3;
    static final double rotateI = 0;
    static final double rotateD = 0;
  }

  private final LEDs leds;
  private final Swerve swerve;
  private final BooleanSupplier targetRightCoralSupplier; // true = right

  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          AlignConstants.translateP,
          AlignConstants.translateI,
          AlignConstants.translateD,
          new TrapezoidProfile.Constraints(
              AlignConstants.maxApproachSpeed, AlignConstants.maxApproachAccel));

  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          AlignConstants.translateP,
          AlignConstants.translateI,
          AlignConstants.translateD,
          new TrapezoidProfile.Constraints(
              AlignConstants.maxApproachSpeed, AlignConstants.maxApproachAccel));

  private final ProfiledPIDController rotController =
      new ProfiledPIDController(
          AlignConstants.rotateP,
          AlignConstants.rotateI,
          AlignConstants.rotateD,
          new TrapezoidProfile.Constraints(
              AlignConstants.maxRotationSpeed, AlignConstants.maxRotationAccel));

  private Pose2d targetPose = new Pose2d();
  private Pose2d closestReefTagPose = new Pose2d();
  private boolean targetRightCoral; // Set during initialize

  Vision vision;

  StructPublisher<Pose2d> targetPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("Vision/AlignTarget", Pose2d.struct)
          .publish();

  private final Command vibrate;

  @Logged(name = "xError", importance = Importance.INFO)
  private double xError = -1;

  @Logged(name = "yError", importance = Importance.INFO)
  private double yError = -1;

  @Logged(name = "rotError", importance = Importance.INFO)
  private double rotError = -1;

  @Logged(name = "xSpeed", importance = Importance.INFO)
  private double xSpeed = -1;

  @Logged(name = "ySpeed", importance = Importance.INFO)
  private double ySpeed = -1;

  @Logged(name = "rotSpeed", importance = Importance.INFO)
  private double rotSpeed = -1;

  /**
   * Creates a command to align with coral of a reef tag.
   *
   * @param leds
   * @param swerve The swerve drive subsystem
   * @param targetRightCoralSupplier Supplies whether to target right coral (true) or left coral
   *     (false)
   */
  public Align(
      Swerve swerve,
      Vision vision,
      BooleanSupplier targetRightCoralSupplier,
      Command vibrate,
      LEDs leds) {
    this.swerve = swerve;
    this.vision = vision;
    this.leds = leds;
    this.targetRightCoralSupplier = targetRightCoralSupplier;
    this.vibrate = vibrate;
    this.vision = vision;

    rotController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = swerve.getPose();
    // vibrate.accept(0.5);

    if (!Align.canAlign(swerve, vision)) {
      this.cancel();
    }

    leds.runPattern(
        LEDPattern.rainbow(255, 128)
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1d / 120d)));

    closestReefTagPose = vision.findClosestReefTag(currentPose);
    // Set tolerances for velocity-based completion
    xController.setTolerance(AlignConstants.positionTolerance, AlignConstants.velocityTolerance);
    yController.setTolerance(AlignConstants.positionTolerance, AlignConstants.velocityTolerance);
    rotController.setTolerance(
        AlignConstants.rotationTolerance, AlignConstants.rotationVelocityTolerance);

    targetRightCoral = targetRightCoralSupplier.getAsBoolean();

    calculateTargetPose();
    targetPosePublisher.set(targetPose);

    // Get current robot pose

    ChassisSpeeds currentSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            swerve.kinematics.toChassisSpeeds(swerve.getModuleStates()),
            swerve.getPose().getRotation());

    // Reset controllers with the current error and target of 0 (no error)
    xController.reset(currentPose.getX(), currentSpeeds.vxMetersPerSecond);
    yController.reset(currentPose.getY(), -currentSpeeds.vyMetersPerSecond);
    rotController.reset(
        currentPose.getRotation().getRadians(), -currentSpeeds.omegaRadiansPerSecond);
  }

  private void calculateTargetPose() {
    double lateralOffset =
        targetRightCoral ? AlignConstants.coralRightOffset : AlignConstants.coralLeftOffset;

    Translation2d lateralOffsetTranslation =
        new Translation2d(0, lateralOffset + AlignConstants.ejectOffset);
    lateralOffsetTranslation = lateralOffsetTranslation.rotateBy(closestReefTagPose.getRotation());

    // Calculate the position that places the front bumper at the tag face
    Translation2d approachOffset = new Translation2d(-AlignConstants.robotCenterToFrontDistance, 0);
    approachOffset = approachOffset.rotateBy(closestReefTagPose.getRotation());

    // Final target pose: tag + lateral offset + approach offset, facing the tag
    targetPose =
        new Pose2d(
            closestReefTagPose.getX() + lateralOffsetTranslation.getX() + approachOffset.getX(),
            closestReefTagPose.getY() + lateralOffsetTranslation.getY() + approachOffset.getY(),
            closestReefTagPose.getRotation().rotateBy(Rotation2d.k180deg));
    if (Constants.devMode) {
      SmartDashboard.putNumber("ReefAlign/TargetX", targetPose.getX());
      SmartDashboard.putNumber("ReefAlign/TargetY", targetPose.getY());
      SmartDashboard.putNumber("ReefAlign/TargetRot", targetPose.getRotation().getDegrees());
    }
  }

  @Override
  public void execute() {

    Pose2d currentPose = swerve.getPose();

    // Calculate errors
    xError = Math.abs(targetPose.getX() - currentPose.getX());
    yError = Math.abs(targetPose.getY() - currentPose.getY());
    rotError = Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getRadians());

    // Calculate outputs by setting goal to 0 (we want zero error)
    xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    rotSpeed =
        rotController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    if (Constants.devMode) {
      SmartDashboard.putNumber("ReefAlign/XError", xError);
      SmartDashboard.putNumber("ReefAlign/YError", yError);
      SmartDashboard.putNumber("ReefAlign/RotError", rotError);
      SmartDashboard.putNumber("ReefAlign/XSpeed", xSpeed);
      SmartDashboard.putNumber("ReefAlign/YSpeed", ySpeed);
      SmartDashboard.putNumber("ReefAlign/RotSpeed", rotSpeed);

      // Also log velocities for debugging
      SmartDashboard.putNumber("ReefAlign/XVelocity", xController.getSetpoint().velocity);
      SmartDashboard.putNumber("ReefAlign/YVelocity", yController.getSetpoint().velocity);
      SmartDashboard.putNumber("ReefAlign/RotVelocity", rotController.getSetpoint().velocity);
    }

    swerve.driveChassisSpeedsRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            -xSpeed, -ySpeed, -rotSpeed, swerve.getPose().getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopDrive();
    vibrate.schedule();
    leds.runPattern(leds.getAllianceLed());
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = swerve.getPose();
    double distanceToTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double rotationError =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

    if (Constants.devMode) {
      SmartDashboard.putNumber("ReefAlign/DistToTarget", distanceToTarget);
      SmartDashboard.putNumber("ReefAlign/RotError", rotationError);
    }

    // Check if velocity is close to zero rather than position at setpoint
    return xController.atGoal() && yController.atGoal() && rotController.atGoal();
    // && rotController.getVelocityError() < AlignConstants.rotationVelocityTolerance;
  }

  public static boolean canAlign(Swerve swerve, Vision vision) {

    Pose2d closestReefTagPose = vision.findClosestReefTag(swerve.getPose());
    Pose2d currentPose = swerve.getPose();

    return (closestReefTagPose.getTranslation().minus(currentPose.getTranslation()).getNorm() < 2
        && vision.hasTarget());
  }
}
