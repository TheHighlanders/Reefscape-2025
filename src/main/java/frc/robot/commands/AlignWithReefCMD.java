package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

import org.photonvision.EstimatedRobotPose;

public class AlignWithReefCMD extends Command {

  /** Constants for the Reef Coral alignment */
  private static final class AlignConstants {
    // IDs of AprilTags on the reef
    // Update with actual reef tag IDs

    // Distance from robot center to front bumper (meters)
    static final double robotCenterToFrontDistance = -0.38;

    // Lateral offset from tag to coral (meters)
    static final double coralLeftOffset = -0.165; // Left coral Y offset (negative = left)
    static final double coralRightOffset = 0.165; // Right coral Y offset (positive = right)

    static final double ejectOffset = 0.25;

    // Position tolerance (meters)
    static final double positionTolerance = 0.005;

    // Rotation tolerance (radians)
    static final double rotationTolerance = 0.05;

    // Maximum approach speed (m/s)
    static final double maxApproachSpeed = 1.5;

    // Maximum rotation speed (rad/s)
    static final double maxRotationSpeed = 1.0;

    static final double translateP = 3; // 2.25;
    static final double translateI = 0.01; // 0.05;
    static final double translateD = 0;

    static final double rotateP = 2; // 1.5;
    static final double rotateI = 0;
    static final double rotateD = 0; // 0.5;
  }

  private final Swerve swerve;
  private final Vision vision;
  private final BooleanSupplier targetRightCoralSupplier; // true = right

  private final PIDController xController =
      new PIDController(
          AlignConstants.translateP, AlignConstants.translateI, AlignConstants.translateD);

  private final PIDController yController =
      new PIDController(
          AlignConstants.translateP, AlignConstants.translateI, AlignConstants.translateD);

  private final PIDController rotController =
      new PIDController(AlignConstants.rotateP, AlignConstants.rotateI, AlignConstants.rotateD);

  private Pose2d targetPose;
  private Pose2d closestReefTagPose;
  private boolean targetRightCoral; // Set during initialize
  private boolean hasTargetTagOnInit = true;

  StructPublisher<Pose2d> targetPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("Vision/AlignTarget", Pose2d.struct)
          .publish();
  private DoubleConsumer vibrate;

  /**
   * Creates a command to align with coral of a reef tag.
   *
   * @param swerve The swerve drive subsystem
   * @param vision The vision subsystem
   * @param targetRightCoralSupplier Supplies whether to target right coral (true) or left coral
   *     (false)
   */
  public AlignWithReefCMD(
      Swerve swerve,
      Vision vision,
      BooleanSupplier targetRightCoralSupplier,
      DoubleConsumer vibrate) {
    this.swerve = swerve;
    this.vision = vision;
    this.targetRightCoralSupplier = targetRightCoralSupplier;

    this.vibrate = vibrate;

    rotController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve, vision);
  }

  @Override
  public void initialize() {
    // vibrate.accept(0.5);

    hasTargetTagOnInit = vision.hasTarget();

    Optional<EstimatedRobotPose> estRobotPose = vision.getEstimatedRobotPose();

    if (hasTargetTagOnInit) {
      if(estRobotPose.isPresent()){
        swerve.resetOdometry(estRobotPose.get().estimatedPose.toPose2d());
      }
    }

    targetRightCoral = targetRightCoralSupplier.getAsBoolean();
    closestReefTagPose = vision.findClosestReefTag(swerve.getPose());
    if (Constants.devMode) {
      SmartDashboard.putNumber("ReefAlign/TagX", closestReefTagPose.getX());
      SmartDashboard.putNumber("ReefAlign/TagY", closestReefTagPose.getY());
      SmartDashboard.putString("ReefAlign/TargetSide", targetRightCoral ? "RIGHT" : "LEFT");
      SmartDashboard.putBoolean("ReefAlign/Started With Vision Target", hasTargetTagOnInit);
    }

    calculateTargetPose();

    targetPosePublisher.set(targetPose);
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
    if (Constants.devMode) {
      SmartDashboard.putNumber("ReefAlign/XSpeed", xSpeed);
      SmartDashboard.putNumber("ReefAlign/YSpeed", ySpeed);
      SmartDashboard.putNumber("ReefAlign/RotSpeed", rotSpeed);
    }

    swerve.driveChassisSpeedsRobotRelative(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            -xSpeed, -ySpeed, -rotSpeed, swerve.getPose().getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopDrive();
    vibrate.accept(0.1);
    SmartDashboard.putBoolean("ReefAlign/Completed", true);
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

    if (distanceToTarget > 1.5 || !hasTargetTagOnInit) {
      return true;
    }

    boolean atTarget =
        distanceToTarget < AlignConstants.positionTolerance
            && rotationError < AlignConstants.rotationTolerance;
    SmartDashboard.putBoolean("ReefAlign/AtTarget", atTarget);

    return atTarget;
  }
}
