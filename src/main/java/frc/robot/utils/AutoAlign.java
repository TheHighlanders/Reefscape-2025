package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

class AutoAlignConstants {
  // IDs of AprilTags on the reef are defined in Vision.java

  // Distance from robot center to front bumper (meters)
  public static final double ROBOT_CENTER_TO_FRONT_DISTANCE = -0.38;

  // Lateral offset from tag to coral (meters)
  public static final double CORAL_LEFT_OFFSET = -0.165; // Left coral Y offset (negative = left)
  public static final double CORAL_RIGHT_OFFSET = 0.165; // Right coral Y offset (positive = right)

  public static final double DEFAULT_EJECTOR_OFFSET = 0.30;

  // Position tolerance (meters)
  public static final double POSITION_TOLERANCE = 0.005;

  // Rotation tolerance (radians)
  public static final double ROTATION_TOLERANCE = 0.05;

  // Maximum approach speed (m/s)
  public static final double MAX_APPROACH_SPEED = 1.5;

  // Maximum rotation speed (rad/s)
  public static final double MAX_ROTATION_SPEED = 1.0;

  // PID constants for translation
  public static final double TRANSLATE_P = 3.3;
  public static final double TRANSLATE_I = 0.01;
  public static final double TRANSLATE_D = 0.0;

  // PID constants for rotation
  public static final double ROTATE_P = 2.0;
  public static final double ROTATE_I = 0.0;
  public static final double ROTATE_D = 0.0;
}

public class AutoAlign {
  /**
   * Calculates the target pose for aligning with a reef coral
   *
   * @param tagPose The pose of the AprilTag on the reef
   * @param targetRightCoral Whether to target the right coral (true) or left coral (false)
   * @param ejectorOffset Custom ejector offset, or use DEFAULT_EJECTOR_OFFSET if 0
   * @return The target pose for the robot to align with the coral
   */
  public static Pose2d calculateReefTargetPose(
      Pose2d tagPose, boolean targetRightCoral, double ejectorOffset) {

    // Use default ejector offset if not specified
    if (ejectorOffset == 0) {
      ejectorOffset = AutoAlignConstants.DEFAULT_EJECTOR_OFFSET;
    }

    // Determine lateral offset based on target side
    double lateralOffset =
        targetRightCoral
            ? AutoAlignConstants.CORAL_RIGHT_OFFSET
            : AutoAlignConstants.CORAL_LEFT_OFFSET;

    // Apply ejector offset to the lateral offset
    lateralOffset += ejectorOffset;

    // Calculate lateral offset translation and rotate by tag rotation
    Translation2d lateralOffsetTranslation = new Translation2d(0, lateralOffset);
    lateralOffsetTranslation = lateralOffsetTranslation.rotateBy(tagPose.getRotation());

    // Calculate approach offset translation and rotate by tag rotation
    Translation2d approachOffset =
        new Translation2d(-AutoAlignConstants.ROBOT_CENTER_TO_FRONT_DISTANCE, 0);
    approachOffset = approachOffset.rotateBy(tagPose.getRotation());

    // Calculate final target pose: tag + lateral offset + approach offset, facing
    // away from tag
    return new Pose2d(
        tagPose.getX() + lateralOffsetTranslation.getX() + approachOffset.getX(),
        tagPose.getY() + lateralOffsetTranslation.getY() + approachOffset.getY(),
        tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
  }

  /** Simplified version that uses the default ejector offset */
  public static Pose2d calculateReefTargetPose(Pose2d tagPose, boolean targetRightCoral) {
    return calculateReefTargetPose(
        tagPose, targetRightCoral, AutoAlignConstants.DEFAULT_EJECTOR_OFFSET);
  }

  /**
   * Checks if the current pose is within tolerance of the target pose
   *
   * @param currentPose Current robot pose
   * @param targetPose Target robot pose
   * @return True if within position and rotation tolerance
   */
  public static boolean isAtTargetPose(Pose2d currentPose, Pose2d targetPose) {
    double distanceToTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double rotationError =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

    return distanceToTarget < AutoAlignConstants.POSITION_TOLERANCE
        && rotationError < AutoAlignConstants.ROTATION_TOLERANCE;
  }

  /**
   * Creates a command to align with a reef coral
   *
   * @param swerve The swerve drive subsystem
   * @param vision The vision subsystem
   * @param targetRightCoralSupplier Supplies whether to target right coral (true) or left coral
   *     (false)
   * @param vibrate Optional controller vibration function
   * @return Command to align with reef coral
   */
  public static Command alignWithReef(
      Swerve swerve,
      Vision vision,
      BooleanSupplier targetRightCoralSupplier,
      DoubleConsumer vibrate) {

    return alignWithReef(
        swerve,
        vision,
        targetRightCoralSupplier,
        vibrate,
        AutoAlignConstants.DEFAULT_EJECTOR_OFFSET);
  }

  /**
   * Creates a command to align with a reef coral with custom ejector offset
   *
   * @param swerve The swerve drive subsystem
   * @param vision The vision subsystem
   * @param targetRightCoralSupplier Supplies whether to target right coral (true) or left coral
   *     (false)
   * @param vibrate Optional controller vibration function
   * @param customEjectorOffset Custom ejector offset to use
   * @return Command to align with reef coral
   */
  public static Command alignWithReef(
      Swerve swerve,
      Vision vision,
      BooleanSupplier targetRightCoralSupplier,
      DoubleConsumer vibrate,
      double customEjectorOffset) {

    // Create PID controllers for motion
    PIDController xController =
        new PIDController(
            AutoAlignConstants.TRANSLATE_P,
            AutoAlignConstants.TRANSLATE_I,
            AutoAlignConstants.TRANSLATE_D);

    PIDController yController =
        new PIDController(
            AutoAlignConstants.TRANSLATE_P,
            AutoAlignConstants.TRANSLATE_I,
            AutoAlignConstants.TRANSLATE_D);

    PIDController rotController =
        new PIDController(
            AutoAlignConstants.ROTATE_P, AutoAlignConstants.ROTATE_I, AutoAlignConstants.ROTATE_D);

    rotController.enableContinuousInput(-Math.PI, Math.PI);

    // Target tracking variables
    final boolean[] targetRightCoral = new boolean[1];
    final Pose2d[] closestReefTagPose = new Pose2d[1];
    final Pose2d[] targetPose = new Pose2d[1];
    final boolean[] hasTargetTagOnInit = new boolean[1];

    return Commands.sequence(
        // Initialize
        Commands.runOnce(
            () -> {
              if (vibrate != null) {
                vibrate.accept(0.5);
              }

              hasTargetTagOnInit[0] = vision.hasTarget();

              // Reset odometry using vision if available
              if (hasTargetTagOnInit[0]) {
                var estRobotPose = vision.getEstimatedRobotPose();
                if (estRobotPose.isPresent()) {
                  swerve.resetOdometry(estRobotPose.get().estimatedPose.toPose2d());
                }
              }

              // Determine target side and find closest reef tag
              targetRightCoral[0] = targetRightCoralSupplier.getAsBoolean();
              closestReefTagPose[0] = vision.findClosestReefTag(swerve.getPose());

              // Calculate target pose
              targetPose[0] =
                  calculateReefTargetPose(
                      closestReefTagPose[0], targetRightCoral[0], customEjectorOffset);
            }),

        // Execute alignment
        Commands.run(
                () -> {
                  Pose2d currentPose = swerve.getPose();

                  // Calculate control outputs
                  double xSpeed = xController.calculate(currentPose.getX(), targetPose[0].getX());
                  double ySpeed = yController.calculate(currentPose.getY(), targetPose[0].getY());
                  double rotSpeed =
                      rotController.calculate(
                          currentPose.getRotation().getRadians(),
                          targetPose[0].getRotation().getRadians());

                  // Clamp speeds to limits
                  xSpeed =
                      MathUtil.clamp(
                          xSpeed,
                          -AutoAlignConstants.MAX_APPROACH_SPEED,
                          AutoAlignConstants.MAX_APPROACH_SPEED);

                  ySpeed =
                      MathUtil.clamp(
                          ySpeed,
                          -AutoAlignConstants.MAX_APPROACH_SPEED,
                          AutoAlignConstants.MAX_APPROACH_SPEED);

                  rotSpeed =
                      MathUtil.clamp(
                          rotSpeed,
                          -AutoAlignConstants.MAX_ROTATION_SPEED,
                          AutoAlignConstants.MAX_ROTATION_SPEED);

                  // Drive robot with calculated speeds
                  swerve.driveChassisSpeedsRobotRelative(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          -xSpeed, -ySpeed, -rotSpeed, swerve.getPose().getRotation()));
                })
            .until(
                () -> {
                  // Check if we're at the target or if we had no vision target at start and moved
                  // too far
                  Pose2d currentPose = swerve.getPose();
                  double distanceToTarget =
                      currentPose.getTranslation().getDistance(targetPose[0].getTranslation());

                  // Abort if too far or no initial vision target
                  if (distanceToTarget > 1.5 || !hasTargetTagOnInit[0]) {
                    return true;
                  }

                  // Check if at target position and rotation
                  return isAtTargetPose(currentPose, targetPose[0]);
                }),

        // End - stop driving
        Commands.runOnce(
            () -> {
              xController.close();
              yController.close();
              rotController.close();
              swerve.stopDrive();
              if (vibrate != null) {
                vibrate.accept(0.1);
              }
            }));
  }
}
