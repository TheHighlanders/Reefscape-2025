package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class AlignWithReefCoral extends Command {

  /** Constants for the Reef Coral alignment */
  private static final class AlignConstants {
    // IDs of AprilTags on the reef
    static final int[] reefTagIds = {1, 2, 3, 4}; // Update with actual reef tag IDs

    // Distance from robot center to front bumper (meters)
    static final double robotCenterToFrontDistance = 0.4;

    // Lateral offset from tag to coral (meters)
    static final double coralLeftOffset = -0.2; // Left coral Y offset (negative = left)
    static final double coralRightOffset = 0.2; // Right coral Y offset (positive = right)

    // Position tolerance (meters)
    static final double positionTolerance = 0.05;

    // Rotation tolerance (radians)
    static final double rotationTolerance = 0.05;

    // Maximum approach speed (m/s)
    static final double maxApproachSpeed = 1.5;

    // Maximum rotation speed (rad/s)
    static final double maxRotationSpeed = 1.0;

    static final double translateP = 1.5;
    static final double translateI = 0.05;
    static final double translateD = 0;

    static final double rotateP = 1.2;
    static final double rotateI = 0;
    static final double rotateD = 0.5;
  }

  private final Swerve swerve;
  private final Vision vision;
  private final BooleanSupplier targetRightCoralSupplier; // true = right
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotController;

  private Pose2d targetPose;
  private Pose2d reefTagPose;
  private boolean hasTarget = false;
  private boolean targetRightCoral; // Set during initialize

  /**
   * Creates a command to align with coral of a reef tag.
   *
   * @param swerve The swerve drive subsystem
   * @param vision The vision subsystem
   * @param targetRightCoralSupplier Supplies whether to target right coral (true) or left coral
   *     (false)
   */
  public AlignWithReefCoral(
      Swerve swerve, Vision vision, BooleanSupplier targetRightCoralSupplier) {
    this.swerve = swerve;
    this.vision = vision;
    this.targetRightCoralSupplier = targetRightCoralSupplier;

    xController =
        new PIDController(
            AlignConstants.translateP, AlignConstants.translateI, AlignConstants.translateD);

    yController =
        new PIDController(
            AlignConstants.translateP, AlignConstants.translateI, AlignConstants.translateD);

    rotController =
        new PIDController(AlignConstants.rotateP, AlignConstants.rotateI, AlignConstants.rotateD);

    rotController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    targetRightCoral = targetRightCoralSupplier.getAsBoolean();

    Optional<Pose2d> closestReefTagPose = vision.findClosestReefTag(swerve.getPose());

    if (closestReefTagPose.isPresent()) {
      reefTagPose = closestReefTagPose.get();
      hasTarget = true;
      SmartDashboard.putBoolean("ReefAlign/HasTarget", true);
      SmartDashboard.putNumber("ReefAlign/TagX", reefTagPose.getX());
      SmartDashboard.putNumber("ReefAlign/TagY", reefTagPose.getY());
      SmartDashboard.putString("ReefAlign/TargetSide", targetRightCoral ? "RIGHT" : "LEFT");

      calculateTargetPose();
    } else {
      hasTarget = false;
      SmartDashboard.putBoolean("ReefAlign/HasTarget", false);
    }
  }

  private void calculateTargetPose() {
    double lateralOffset =
        targetRightCoral ? AlignConstants.coralRightOffset : AlignConstants.coralLeftOffset;

    Translation2d lateralOffsetTranslation = new Translation2d(0, lateralOffset);
    lateralOffsetTranslation = lateralOffsetTranslation.rotateBy(reefTagPose.getRotation());

    // Calculate the position that places the front bumper at the tag face
    Translation2d approachOffset = new Translation2d(-AlignConstants.robotCenterToFrontDistance, 0);
    approachOffset = approachOffset.rotateBy(reefTagPose.getRotation());

    // Final target pose: tag + lateral offset + approach offset, facing the tag
    targetPose =
        new Pose2d(
            reefTagPose.getX() + lateralOffsetTranslation.getX() + approachOffset.getX(),
            reefTagPose.getY() + lateralOffsetTranslation.getY() + approachOffset.getY(),
            reefTagPose.getRotation());

    SmartDashboard.putNumber("ReefAlign/TargetX", targetPose.getX());
    SmartDashboard.putNumber("ReefAlign/TargetY", targetPose.getY());
    SmartDashboard.putNumber("ReefAlign/TargetRot", targetPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    if (!hasTarget) {
      swerve.stopDrive();
      return;
    }

    Pose2d currentPose = swerve.getPose();

    double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    double rotSpeed =
        rotController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    xSpeed =
        MathUtil.clamp(xSpeed, -AlignConstants.maxApproachSpeed, AlignConstants.maxApproachSpeed);
    ySpeed =
        MathUtil.clamp(ySpeed, -AlignConstants.maxApproachSpeed, AlignConstants.maxApproachSpeed);
    rotSpeed =
        MathUtil.clamp(rotSpeed, -AlignConstants.maxRotationSpeed, AlignConstants.maxRotationSpeed);

    SmartDashboard.putNumber("ReefAlign/XSpeed", xSpeed);
    SmartDashboard.putNumber("ReefAlign/YSpeed", ySpeed);
    SmartDashboard.putNumber("ReefAlign/RotSpeed", rotSpeed);

    swerve.driveChassisSpeedsRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotSpeed, swerve.getPose().getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopDrive();
    SmartDashboard.putBoolean("ReefAlign/Completed", true);
  }

  @Override
  public boolean isFinished() {
    if (!hasTarget) {
      return true;
    }

    Pose2d currentPose = swerve.getPose();
    double distanceToTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    double rotationError =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

    boolean atTarget =
        distanceToTarget < AlignConstants.positionTolerance
            && rotationError < AlignConstants.rotationTolerance;

    SmartDashboard.putNumber("ReefAlign/DistToTarget", distanceToTarget);
    SmartDashboard.putNumber("ReefAlign/RotError", rotationError);
    SmartDashboard.putBoolean("ReefAlign/AtTarget", atTarget);

    return atTarget;
  }
}
