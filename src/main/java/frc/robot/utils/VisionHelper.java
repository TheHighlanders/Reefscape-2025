package frc.robot.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Utility class for vision processing and pose estimation */
public class VisionHelper {
  // Standard deviations based on tag count and distance
  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 0.9);
  public static final Matrix<N3, N1> DOUBLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 0.6);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.2, 0.2, 0.4);
  public static final Matrix<N3, N1> INFINITE_STD_DEVS =
      VecBuilder.fill(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

  // Tag distance factors
  private static final double DISTANCE_FACTOR = 2.0; // Increase std devs as distance increases
  private static final double MAX_SINGLE_TAG_DISTANCE =
      4.0; // Maximum reliable distance for single tag

  // Ambiguity thresholds
  private static final double MAX_ACCEPTABLE_AMBIGUITY = 0.2;

  /**
   * Calculate standard deviations for a vision measurement
   *
   * @param estimatedPose The estimated pose
   * @param targets The tags used for estimation
   * @return Standard deviation matrix for x, y, and theta
   */
  public static Matrix<N3, N1> calculateVisionStdDevs(
      EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {

    // If no targets, return infinite standard deviations
    if (targets == null || targets.isEmpty()) {
      return INFINITE_STD_DEVS;
    }

    // Calculate average distance to tags
    double avgDistance = 0;
    for (var target : targets) {
      avgDistance += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    avgDistance /= targets.size();

    // Apply distance scaling factor (increase uncertainty with distance)
    double distanceFactor = 1.0 + (avgDistance * avgDistance / 30.0);

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

    // Scale by distance factor
    return baseStdDevs.times(distanceFactor);
  }

  /**
   * Validate a vision measurement to determine if it should be used or rejected
   *
   * @param pose The estimated robot pose
   * @param targets The list of tracked targets used for estimation
   * @return True if the measurement is valid, false otherwise
   */
  public static boolean isVisionMeasurementValid(Pose3d pose, List<PhotonTrackedTarget> targets) {
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
    }

    return true;
  }

  /**
   * Process and validate pipeline results to estimate robot pose with appropriate confidence
   *
   * @param results The list of PhotonVision pipeline results
   * @param poseStrategy The pose estimation strategy used
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
    boolean isValid =
        isVisionMeasurementValid(
            estimatedPose.get().estimatedPose, estimatedPose.get().targetsUsed);

    if (!isValid) {
      return INFINITE_STD_DEVS;
    }

    // Calculate standard deviations based on pose estimate
    return calculateVisionStdDevs(estimatedPose.get(), estimatedPose.get().targetsUsed);
  }

  public static Matrix<N3, N1> processPipelineResult(
      PhotonPipelineResult result, Optional<EstimatedRobotPose> estimatedPose) {

    // Call the list version with a single-item list
    return processPipelineResult(List.of(result), estimatedPose);
  }

  /**
   * Filter targets based on ambiguity and distance
   *
   * @param targets List of targets to filter
   * @param maxDistance Maximum acceptable distance
   * @param maxAmbiguity Maximum acceptable ambiguity
   * @return Filtered list of targets
   */
  public static List<PhotonTrackedTarget> filterTargets(
      List<PhotonTrackedTarget> targets, double maxDistance, double maxAmbiguity) {

    // Create a new list for filtered targets
    List<PhotonTrackedTarget> filteredTargets = new java.util.ArrayList<>();

    for (PhotonTrackedTarget target : targets) {
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
}
