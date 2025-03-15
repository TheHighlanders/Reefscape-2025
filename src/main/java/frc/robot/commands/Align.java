package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
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
    static final double positionTolerance = 0.05;

    // Velocity tolerance (meters/s)
    static final double velocityTolerance = 0.1;

    // Rotation tolerance (radians)
    static final double rotationTolerance = 0;

    // Rotation velocity tolerance (rad/s)
    static final double rotationVelocityTolerance = 0.05;

    // Maximum approach speed (m/s)
    static final double maxApproachSpeed = 5;
    static final double maxApproachAccel = 5.0;

    // Maximum rotation speed (rad/s)
    static final double maxRotationSpeed = 5;
    static final double maxRotationAccel = 1.5;

    static final double translateP = 5;
    static final double translateI = 0.01;
    static final double translateD = 0;

    static final double rotateP = 3;
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
  Vision vision;

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
  public Align(Swerve swerve, Vision vision, BooleanSupplier targetRightCoralSupplier, Command vibrate) {
    this.swerve = swerve;
    this.targetRightCoralSupplier = targetRightCoralSupplier;
    this.vibrate = vibrate;
    this.vision = vision;

    rotController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = swerve.getPose();

    closestReefTagPose = vision.findClosestReefTag(currentPose);

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

    // Reset controllers with the current error and target of 0 (no error)
    xController.reset(currentPose.getX(), 0);
    yController.reset(currentPose.getY(), 0);
    rotController.reset(currentPose.getRotation().getRadians(), 0);
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
    double xError = Math.abs(targetPose.getX() - currentPose.getX());
    double yError = Math.abs(targetPose.getY() - currentPose.getY());
    double rotationError =
        Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getRadians());

    // Calculate outputs by setting goal to 0 (we want zero error)
    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    double rotSpeed =
        rotController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Limit speeds to max values
    // xSpeed =
    //     MathUtil.clamp(xSpeed, -AlignConstants.maxApproachSpeed,
    // AlignConstants.maxApproachSpeed);
    // ySpeed =
    //     MathUtil.clamp(ySpeed, -AlignConstants.maxApproachSpeed,
    // AlignConstants.maxApproachSpeed);
    // rotSpeed =
    //     MathUtil.clamp(rotSpeed, -AlignConstants.maxRotationSpeed,
    // AlignConstants.maxRotationSpeed);

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
    return xController.atGoal() && yController.atGoal() && rotController.getVelocityError()<AlignConstants.rotationVelocityTolerance;
  }
}
