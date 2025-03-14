package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;

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
    static final double positionTolerance = 0.01;

    // Velocity tolerance (meters/s)
    static final double velocityTolerance = 0.05;

    // Rotation tolerance (radians)
    static final double rotationTolerance = 0.01;

    // Rotation velocity tolerance (rad/s)
    static final double rotationVelocityTolerance = 0.05;

    // Maximum approach speed (m/s)
    static final double maxApproachSpeed = 1.5;
    static final double maxApproachAccel = 2.0;

    // Maximum rotation speed (rad/s)
    static final double maxRotationSpeed = 1.0;
    static final double maxRotationAccel = 1.5;

    static final double translateP = 3;
    static final double translateI = 0.01;
    static final double translateD = 0;

    static final double rotateP = 2;
    static final double rotateI = 0;
    static final double rotateD = 0;
  }

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

  private Pose2d targetPose;
  private Pose2d closestReefTagPose;
  private boolean targetRightCoral; // Set during initialize
  private boolean hasTargetTagOnInit = true;

  StructPublisher<Pose2d> targetPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("Vision/AlignTarget", Pose2d.struct)
          .publish();
  private final Command vibrate;

  /**
   * Creates a command to align with coral of a reef tag.
   *
   * @param swerve The swerve drive subsystem
   * @param targetRightCoralSupplier Supplies whether to target right coral (true) or left coral
   *     (false)
   */
  public Align(Swerve swerve, BooleanSupplier targetRightCoralSupplier, Command vibrate) {
    this.swerve = swerve;
    this.targetRightCoralSupplier = targetRightCoralSupplier;
    this.vibrate = vibrate;

    rotController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // Set tolerances for velocity-based completion
    xController.setTolerance(AlignConstants.positionTolerance, AlignConstants.velocityTolerance);
    yController.setTolerance(AlignConstants.positionTolerance, AlignConstants.velocityTolerance);
    rotController.setTolerance(
        AlignConstants.rotationTolerance, AlignConstants.rotationVelocityTolerance);

    hasTargetTagOnInit = true;
    targetRightCoral = targetRightCoralSupplier.getAsBoolean();

    if (Constants.devMode) {
      SmartDashboard.putNumber("ReefAlign/TagX", closestReefTagPose.getX());
      SmartDashboard.putNumber("ReefAlign/TagY", closestReefTagPose.getY());
      SmartDashboard.putString("ReefAlign/TargetSide", targetRightCoral ? "RIGHT" : "LEFT");
      SmartDashboard.putBoolean("ReefAlign/Started With Vision Target", hasTargetTagOnInit);
    }

    calculateTargetPose();
    targetPosePublisher.set(targetPose);

    // Get current robot pose
    Pose2d currentPose = swerve.getPose();

    // Reset PID controllers with current values
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double rotationError = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();

    // Reset controllers with the current error and target of 0 (no error)
    xController.reset(xError, 0);
    yController.reset(yError, 0);
    rotController.reset(rotationError, 0);

    yController.setTolerance(0.01, 0.01);
    xController.setTolerance(0.01, 0.01);
    rotController.setTolerance(0.01);
  }

  private void calculateTargetPose() {

    double lateralOffset =
        targetRightCoral ? AlignConstants.coralRightOffset : AlignConstants.coralLeftOffset;

    Transform2d lateralTransform =
        new Transform2d(
            new Translation2d(0, lateralOffset + AlignConstants.ejectOffset), new Rotation2d(0));

    Transform2d approachTransform =
        new Transform2d(
            new Translation2d(AlignConstants.robotCenterToFrontDistance, 0), new Rotation2d(0));

    // Apply transforms to get final target pose
    // First rotate the transforms to match the tag orientation
    lateralTransform =
        new Transform2d(
            lateralTransform.getTranslation().rotateBy(closestReefTagPose.getRotation()),
            lateralTransform.getRotation());

    approachTransform =
        new Transform2d(
            approachTransform.getTranslation().rotateBy(closestReefTagPose.getRotation()),
            approachTransform.getRotation());

    // Apply transforms to get target pose
    Pose2d offsetPose = closestReefTagPose.transformBy(lateralTransform);
    targetPose = offsetPose.transformBy(approachTransform);

    // Set target rotation (robot facing tag)
    targetPose =
        new Pose2d(
            targetPose.getTranslation(),
            closestReefTagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));

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
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double rotationError = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();

    // Calculate outputs by setting goal to 0 (we want zero error)
    double xSpeed = xController.calculate(xError, 0);
    double ySpeed = yController.calculate(yError, 0);
    double rotSpeed = rotController.calculate(rotationError, 0);

    // Limit speeds to max values
    xSpeed =
        MathUtil.clamp(xSpeed, -AlignConstants.maxApproachSpeed, AlignConstants.maxApproachSpeed);
    ySpeed =
        MathUtil.clamp(ySpeed, -AlignConstants.maxApproachSpeed, AlignConstants.maxApproachSpeed);
    rotSpeed =
        MathUtil.clamp(rotSpeed, -AlignConstants.maxRotationSpeed, AlignConstants.maxRotationSpeed);

    if (Constants.devMode) {
      SmartDashboard.putNumber("ReefAlign/XError", xError);
      SmartDashboard.putNumber("ReefAlign/YError", yError);
      SmartDashboard.putNumber("ReefAlign/RotError", rotationError);
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
    SmartDashboard.putBoolean("ReefAlign/Completed", true);
  }

  @Override
  public boolean isFinished() {
    if (!hasTargetTagOnInit) {
      return true;
    }

    // Check if velocity is close to zero rather than position at setpoint
    return xController.atGoal() && yController.atGoal() && rotController.atGoal();
  }
}
