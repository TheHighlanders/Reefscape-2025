// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants;

/** Utility class for vision processing and pose estimation */
public class VisionHelper {
  // Standard deviations based on tag count

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 0.9);
  public static final Matrix<N3, N1> DOUBLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 0.6);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.2, 0.2, 0.4);
  public static final Matrix<N3, N1> INFINITE_STD_DEVS =
      VecBuilder.fill(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

  // Tag distance factors
  private static final double DISTANCE_FACTOR = 2.0;
  private static final double MAX_SINGLE_TAG_DISTANCE = 4.0;

  // Ambiguity thresholds
  private static final double MAX_ACCEPTABLE_AMBIGUITY = 0.2;

  /**
   * Updates the estimated position of the robot with appropriate confidence levels.
   *
   * @param cameraResult The latest pipeline result from the camera.
   * @param cameraMatrix Camera calibration data
   * @param distCoeffs Camera calibration data
   * @param strat The selected pose solving strategy.
   * @param robotToCamera Transform3d from the center of the robot to the camera
   * @param bestTF Best transform (used for MULTI_TAG_PNP_ON_COPROCESSOR strategy)
   * @return An Optional containing the estimated robot pose if valid
   */
  public static Optional<EstimatedRobotPose> update(
      PhotonPipelineResult cameraResult,
      Matrix<N3, N3> cameraMatrix,
      Matrix<N8, N1> distCoeffs,
      PoseStrategy strat,
      Transform3d robotToCamera,
      Transform3d bestTF) {

    List<PhotonTrackedTarget> filteredTargets =
        filterTargets(cameraResult.getTargets(), MAX_SINGLE_TAG_DISTANCE, MAX_ACCEPTABLE_AMBIGUITY);

    // If no targets after filtering, return empty
    if (filteredTargets.isEmpty()) {
      return Optional.empty();
    }

    // Create a new pipeline result with filtered targets using the original
    // metadata
    PhotonPipelineResult filteredResult =
        new PhotonPipelineResult(
            cameraResult.metadata, filteredTargets, cameraResult.getMultiTagResult());

    Optional<EstimatedRobotPose> estimatedPose;

    switch (strat) {
      case LOWEST_AMBIGUITY ->
          estimatedPose = lowestAmbiguityStrategy(filteredResult, robotToCamera);
      case MULTI_TAG_PNP_ON_RIO ->
          estimatedPose =
              multiTagOnRioStrategy(
                  filteredResult,
                  Optional.of(cameraMatrix),
                  Optional.of(distCoeffs),
                  PoseStrategy.LOWEST_AMBIGUITY,
                  robotToCamera);
      case MULTI_TAG_PNP_ON_COPROCESSOR ->
          estimatedPose =
              multiTagOnCoprocStrategy(
                  filteredResult,
                  Optional.of(cameraMatrix),
                  Optional.of(distCoeffs),
                  robotToCamera,
                  PoseStrategy.LOWEST_AMBIGUITY,
                  bestTF);
      case PNP_DISTANCE_TRIG_SOLVE ->
          estimatedPose =
              trigSolveStrategy(
                  filteredResult,
                  Optional.of(cameraMatrix),
                  Optional.of(distCoeffs),
                  robotToCamera,
                  PoseStrategy.LOWEST_AMBIGUITY,
                  bestTF);
      default -> {
        DriverStation.reportError(
            "[PhotonPoseEstimator] Unknown Position Estimation Strategy!", false);
        return Optional.empty();
      }
    }

    // Validate the estimated pose
    if (estimatedPose.isPresent() && !isVisionMeasurementValid(estimatedPose.get())) {
      return Optional.empty();
    }

    return estimatedPose;
  }

  /** Runs SolvePNP on the roborio */
  private static Optional<EstimatedRobotPose> multiTagOnRioStrategy(
      PhotonPipelineResult result,
      Optional<Matrix<N3, N3>> cameraMatrix,
      Optional<Matrix<N8, N1>> distCoeffs,
      PoseStrategy multiTagFallbackStrategy,
      Transform3d robotToCamera) {

    boolean hasCalibData = cameraMatrix.isPresent() && distCoeffs.isPresent();

    // Cannot run multitagPNP, use fallback strategy
    if (!hasCalibData || result.getTargets().size() < 2) {
      return update(
          result,
          cameraMatrix.orElseThrow(),
          distCoeffs.orElseThrow(),
          multiTagFallbackStrategy,
          robotToCamera,
          new Transform3d());
    }

    var pnpResult =
        VisionEstimation.estimateCamPosePNP(
            cameraMatrix.get(),
            distCoeffs.get(),
            result.getTargets(),
            Constants.getFieldTagLayout(),
            TargetModel.kAprilTag36h11);

    // Try fallback strategy if solvePNP fails for some reason
    if (pnpResult.isEmpty()) {
      return update(
          result,
          cameraMatrix.get(),
          distCoeffs.get(),
          multiTagFallbackStrategy,
          robotToCamera,
          new Transform3d());
    }

    var best =
        new Pose3d()
            .plus(pnpResult.get().best) // field-to-camera
            .plus(robotToCamera.inverse());

    return Optional.of(
        new EstimatedRobotPose(
            best,
            result.getTimestampSeconds(),
            result.getTargets(),
            PoseStrategy.MULTI_TAG_PNP_ON_RIO));
  }

  /** Runs SolvePNP on a coprocessor */
  private static Optional<EstimatedRobotPose> multiTagOnCoprocStrategy(
      PhotonPipelineResult result,
      Optional<Matrix<N3, N3>> cameraMatrix,
      Optional<Matrix<N8, N1>> distCoeffs,
      Transform3d robotToCamera,
      PoseStrategy multiTagFallbackStrategy,
      Transform3d bestTF) {

    if (!bestTF.equals(new Transform3d())) {
      var best_tf = bestTF;
      var best =
          new Pose3d()
              .plus(best_tf) // field-to-camera
              .relativeTo(Constants.getFieldTagLayout().getOrigin())
              .plus(robotToCamera.inverse()); // field-to-robot

      return Optional.of(
          new EstimatedRobotPose(
              best,
              result.getTimestampSeconds(),
              result.getTargets(),
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
    } else {
      return update(
          result,
          cameraMatrix.orElseThrow(),
          distCoeffs.orElseThrow(),
          multiTagFallbackStrategy,
          robotToCamera,
          new Transform3d());
    }
  }

  private static Optional<EstimatedRobotPose> trigSolveStrategy(
      PhotonPipelineResult result,
      Optional<Matrix<N3, N3>> cameraMatrix,
      Optional<Matrix<N8, N1>> distCoeffs,
      Transform3d robotToCamera,
      PoseStrategy fallbackStrategy,
      Transform3d bestTF) {

    // If no heading data is available or the transform is empty, use fallback strategy
    if (bestTF.equals(new Transform3d())) {
      return update(
          result,
          cameraMatrix.orElseThrow(),
          distCoeffs.orElseThrow(),
          fallbackStrategy,
          robotToCamera,
          bestTF);
    }

    // Similar to multiTagOnCoprocStrategy but incorporating heading data
    var best_tf = bestTF;
    var best =
        new Pose3d()
            .plus(best_tf) // field-to-camera
            .relativeTo(Constants.getFieldTagLayout().getOrigin())
            .plus(robotToCamera.inverse()); // field-to-robot

    return Optional.of(
        new EstimatedRobotPose(
            best,
            result.getTimestampSeconds(),
            result.getTargets(),
            PoseStrategy.PNP_DISTANCE_TRIG_SOLVE));
  }

  /** Return the estimated position of the robot with the lowest position ambiguity */
  private static Optional<EstimatedRobotPose> lowestAmbiguityStrategy(
      PhotonPipelineResult result, Transform3d robotToCamera) {

    PhotonTrackedTarget lowestAmbiguityTarget = null;
    double lowestAmbiguityScore = 10;

    for (PhotonTrackedTarget target : result.targets) {
      double targetPoseAmbiguity = target.getPoseAmbiguity();

      // Reject if not a tag on this field
      if (target.getFiducialId() < 1 || target.getFiducialId() > 22) {
        continue;
      }

      // Reject if too far
      if (target.getBestCameraToTarget().getTranslation().getNorm() > MAX_SINGLE_TAG_DISTANCE) {
        continue;
      }

      // Make sure the target is a Fiducial target.
      if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
        lowestAmbiguityScore = targetPoseAmbiguity;
        lowestAmbiguityTarget = target;
      }
    }

    // Although there are confirmed to be targets, none of them may be fiducial
    // targets.
    if (lowestAmbiguityTarget == null) {
      return Optional.empty();
    }
    int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

    Optional<Pose3d> targetPosition = Constants.getFieldTagLayout().getTagPose(targetFiducialId);

    if (targetPosition.isEmpty()) {
      DriverStation.reportError(
          "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: " + targetFiducialId,
          false);
      return Optional.empty();
    }

    var estimatedRobotPose =
        new EstimatedRobotPose(
            targetPosition
                .get()
                .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                .transformBy(robotToCamera.inverse()),
            result.getTimestampSeconds(),
            result.getTargets(),
            PoseStrategy.LOWEST_AMBIGUITY);

    return Optional.of(estimatedRobotPose);
  }

  /**
   * Calculate standard deviations for a vision measurement
   *
   * @param estimation The estimated robot pose with targets used
   * @return Standard deviation matrix for x, y, and theta
   */
  public static Matrix<N3, N1> findVisionMeasurementStdDevs(EstimatedRobotPose estimation) {
    List<PhotonTrackedTarget> targets = estimation.targetsUsed;

    // If no targets, return infinite standard deviations
    if (targets == null || targets.isEmpty()) {
      return INFINITE_STD_DEVS;
    }

    // Calculate average distance to tags
    double sumDistance = 0;
    for (var target : targets) {
      var t3d = target.getBestCameraToTarget();
      sumDistance +=
          Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
    }
    double avgDistance = sumDistance / targets.size();

    // Select base standard deviations based on number of tags
    Matrix<N3, N1> baseStdDevs;
    if (targets.size() >= 3) {
      baseStdDevs = MULTI_TAG_STD_DEVS;
    } else if (targets.size() == 2) {
      baseStdDevs = DOUBLE_TAG_STD_DEVS;
    } else {
      // For single tag, check pose ambiguity
      if (targets.get(0).getPoseAmbiguity() > MAX_ACCEPTABLE_AMBIGUITY) {
        return INFINITE_STD_DEVS;
      }
      // Reject if single tag is too far away
      if (avgDistance > MAX_SINGLE_TAG_DISTANCE) {
        return INFINITE_STD_DEVS;
      }
      baseStdDevs = SINGLE_TAG_STD_DEVS;
    }

    // Apply distance scaling factor
    double distanceFactor = 1.0 + (avgDistance * DISTANCE_FACTOR / 10.0);
    return baseStdDevs.times(distanceFactor);
  }

  /**
   * Validate a vision measurement to determine if it should be used or rejected
   *
   * @param estimatedPose The estimated robot pose
   * @return True if the measurement is valid, false otherwise
   */
  public static boolean isVisionMeasurementValid(EstimatedRobotPose estimatedPose) {
    List<PhotonTrackedTarget> targets = estimatedPose.targetsUsed;
    Pose3d pose = estimatedPose.estimatedPose;

    // No targets? Invalid.
    if (targets == null || targets.isEmpty()) {
      return false;
    }

    // Reject measurements that place robot significantly off the ground
    if (Math.abs(pose.getZ()) > 0.15) {
      return false;
    }

    // With only one tag, apply stricter validation
    if (targets.size() == 1) {
      PhotonTrackedTarget target = targets.get(0);

      // Reject if ambiguity is too high
      if (target.getPoseAmbiguity() > MAX_ACCEPTABLE_AMBIGUITY) {
        return false;
      }

      // Reject if single tag is too far away
      double distance = target.getBestCameraToTarget().getTranslation().getNorm();
      if (distance > MAX_SINGLE_TAG_DISTANCE) {
        return false;
      }

      // Reject if not a valid tag on this field
      if (target.getFiducialId() < 1 || target.getFiducialId() > 22) {
        return false;
      }
    }

    return true;
  }

  /**
   * Filter targets based on ambiguity and distance criteria
   *
   * @param targets List of targets to filter
   * @param maxDistance Maximum acceptable distance
   * @param maxAmbiguity Maximum acceptable ambiguity
   * @return Filtered list of targets
   */
  public static List<PhotonTrackedTarget> filterTargets(
      List<PhotonTrackedTarget> targets, double maxDistance, double maxAmbiguity) {

    if (targets == null || targets.isEmpty()) {
      return new ArrayList<>();
    }

    List<PhotonTrackedTarget> filteredTargets = new ArrayList<>();

    for (PhotonTrackedTarget target : targets) {
      // Skip targets with invalid IDs
      if (target.getFiducialId() < 1 || target.getFiducialId() > 22) {
        continue;
      }

      // Skip targets with high ambiguity
      if (target.getPoseAmbiguity() > maxAmbiguity) {
        continue;
      }

      // Skip targets that are too far away
      double distance = target.getBestCameraToTarget().getTranslation().getNorm();
      if (distance > maxDistance) {
        continue;
      }

      filteredTargets.add(target);
    }

    return filteredTargets;
  }

  /**
   * Process and validate pipeline results to estimate robot pose with appropriate confidence
   *
   * @param results The list of PhotonVision pipeline results
   * @param estimatedPose The estimated pose from PhotonVision
   * @return A matrix of standard deviations representing confidence in the pose estimate
   */
  public static Matrix<N3, N1> processPipelineResult(
      List<PhotonPipelineResult> results, Optional<EstimatedRobotPose> estimatedPose) {

    // If no pose estimate, return infinite standard deviations
    if (estimatedPose.isEmpty()) {
      return INFINITE_STD_DEVS;
    }

    // Check if we have any results with targets
    boolean hasTargets = results.stream().anyMatch(result -> !result.getTargets().isEmpty());
    if (!hasTargets) {
      return INFINITE_STD_DEVS;
    }

    // Check if pose is valid
    boolean isValid = isVisionMeasurementValid(estimatedPose.get());
    if (!isValid) {
      return INFINITE_STD_DEVS;
    }

    // Calculate standard deviations based on pose estimate
    return findVisionMeasurementStdDevs(estimatedPose.get());
  }

  /**
   * Process a single pipeline result
   *
   * @param result The PhotonVision pipeline result
   * @param estimatedPose The estimated pose from PhotonVision
   * @return A matrix of standard deviations representing confidence in the pose estimate
   */
  public static Matrix<N3, N1> processPipelineResult(
      PhotonPipelineResult result, Optional<EstimatedRobotPose> estimatedPose) {

    // Call the list version with a single-item list
    return processPipelineResult(List.of(result), estimatedPose);
  }

  /** Record containing vision measurement and its confidence */
  public static record VisionMeasurement(
      EstimatedRobotPose estimation, Matrix<N3, N1> confidence) {}
}
