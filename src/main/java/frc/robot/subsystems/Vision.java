// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.VisionHelper;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

final class VisionConstants {
  static final PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

  static final Transform3d frontFacingCam =
      new Transform3d(
          new Translation3d(0.205486, -0.122174, 0.4376928),
          new Rotation3d(0, Units.degreesToRadians(25.5), 0));

  static final int[] reefTagIds = {
    6, 7, 8, 9, 10, 11, // RED
    17, 18, 19, 20, 21, 22 // BLUE
  };

  static final double cameraFPS = 20;
  static final double debounceFrames = Math.ceil(50 / cameraFPS);

  // Maximum acceptable distance for reliable tag detection
  static final double MAX_TAG_DISTANCE = 6.0; // meters

  // Maximum acceptable ambiguity for pose estimation
  static final double MAX_POSE_AMBIGUITY = 0.2;
}

public class Vision extends SubsystemBase {

  private static final CameraConfig[] CAMERA_CONFIGS = {
    new CameraConfig("ReefCamera", VisionConstants.frontFacingCam)
    // Add more cameras as needed
    // Example: new CameraConfig("BackCamera", new Transform3d(...))
  };

  // Internal camera management
  private PhotonCamera[] cameras;
  private PhotonPoseEstimator[] poseEstimators;
  private Pose3d[] cameraPoses;
  private String[] cameraNames;

  private AprilTagFieldLayout aprilTagFieldLayout;
  private Matrix<N3, N1> stdDev = new Matrix<N3, N1>(Nat.N3(), Nat.N1());
  private List<Pose2d> reefTagPoses = new ArrayList<>();
  private boolean hasTarget = false;
  private int frameCounter = 0;

  // For publishing best estimate to NetworkTables
  StructPublisher<Pose2d> visionEstPose =
      NetworkTableInstance.getDefault()
          .getStructTopic("/Vision/Vision Estimated Pose", Pose2d.struct)
          .publish();

  /** Creates a new Vision. */
  public Vision() {
    // Load the AprilTag field layout
    this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    // Initialize camera arrays
    int cameraCount = CAMERA_CONFIGS.length;
    cameras = new PhotonCamera[cameraCount];
    poseEstimators = new PhotonPoseEstimator[cameraCount];
    cameraPoses = new Pose3d[cameraCount];
    cameraNames = new String[cameraCount];

    // Initialize all cameras and pose estimators
    for (int i = 0; i < cameraCount; i++) {
      cameraNames[i] = CAMERA_CONFIGS[i].name;
      cameras[i] = new PhotonCamera(cameraNames[i]);

      poseEstimators[i] =
          new PhotonPoseEstimator(
              aprilTagFieldLayout, VisionConstants.poseStrategy, CAMERA_CONFIGS[i].transform);
      poseEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      cameraPoses[i] = Pose3d.kZero;
    }

    // Store reef tag poses for quick lookup
    for (int tagId : VisionConstants.reefTagIds) {
      var tagPoseOptional = aprilTagFieldLayout.getTagPose(tagId);
      if (tagPoseOptional.isPresent()) {
        reefTagPoses.add(tagPoseOptional.get().toPose2d());
      }
    }

    // Initialize standard deviation with infinite values
    stdDev = VisionHelper.INFINITE_STD_DEVS;
  }

  @Override
  public void periodic() {
    // Process all cameras
    Optional<EstimatedRobotPose> bestEstimate = Optional.empty();
    double bestConfidence = Double.POSITIVE_INFINITY;

    for (int i = 0; i < cameras.length; i++) {
      // Get vision data from this camera
      Optional<EstimatedRobotPose> camEstimate = getEstimatedRobotPose(i);

      // If we got a valid estimate, check if it's better than our current best
      if (camEstimate.isPresent()) {
        // Process the pipeline result to calculate confidence
        List<PhotonPipelineResult> result = cameras[i].getAllUnreadResults();
        if (result.isEmpty()) {
          continue;
        }
        Matrix<N3, N1> camStdDev = VisionHelper.processPipelineResult(result, camEstimate);

        // Calculate overall confidence metric (lower is better)
        double confidence = camStdDev.get(0, 0) + camStdDev.get(1, 0) + camStdDev.get(2, 0);

        // Update our best estimate if this one is better
        if (confidence < bestConfidence) {
          bestEstimate = camEstimate;
          bestConfidence = confidence;
          stdDev = camStdDev;
          cameraPoses[i] = camEstimate.get().estimatedPose;
        }

        // Log individual camera data
        SmartDashboard.putNumber(
            "Vision/" + cameraNames[i] + "/Target Count", camEstimate.get().targetsUsed.size());
        SmartDashboard.putNumber("Vision/" + cameraNames[i] + "/Confidence", confidence);
      }
    }

    // Process the best estimate
    if (bestEstimate.isPresent()) {
      frameCounter = 0;
      hasTarget = true;

      // Publish the estimated pose
      visionEstPose.set(bestEstimate.get().estimatedPose.toPose2d());

      // Log debug information
      SmartDashboard.putNumber("Vision/Target Count", bestEstimate.get().targetsUsed.size());

      if (!bestEstimate.get().targetsUsed.isEmpty()) {
        SmartDashboard.putNumber(
            "Vision/Average Distance", calculateAverageDistance(bestEstimate.get().targetsUsed));

        // Log first target's info
        PhotonTrackedTarget firstTarget = bestEstimate.get().targetsUsed.get(0);
        SmartDashboard.putNumber("Vision/FirstTarget/FiducialID", firstTarget.getFiducialId());
        SmartDashboard.putNumber("Vision/FirstTarget/Ambiguity", firstTarget.getPoseAmbiguity());
      }
    } else {
      // If we've lost the target for too many frames, mark as no target
      if (frameCounter > VisionConstants.debounceFrames) {
        hasTarget = false;
        stdDev = VisionHelper.INFINITE_STD_DEVS;
      } else {
        frameCounter++;
      }
    }

    // Log target status
    SmartDashboard.putBoolean("Vision/Has Target", hasTarget);

    // Log standard deviations (confidence values)
    if (stdDev.getNumRows() >= 3) {
      SmartDashboard.putNumber("Vision/StdDev/X", stdDev.get(0, 0));
      SmartDashboard.putNumber("Vision/StdDev/Y", stdDev.get(1, 0));
      SmartDashboard.putNumber("Vision/StdDev/Theta", stdDev.get(2, 0));
    }
  }

  /**
   * Get the estimated robot pose from a specific camera
   *
   * @param cameraIndex The index of the camera to use
   * @return Optional containing the estimated robot pose from the specified camera, or empty if no
   *     valid estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedRobotPose(int cameraIndex) {
    // Check if the index is valid
    if (cameraIndex < 0 || cameraIndex >= cameraPoses.length) {
      return Optional.empty();
    }

    // Check if this camera has a valid pose
    if (cameraPoses[cameraIndex] != Pose3d.kZero) {
      return Optional.of(
          new EstimatedRobotPose(
              cameraPoses[cameraIndex],
              Timer.getFPGATimestamp(),
              new ArrayList<>(), // Ideally, we'd store the actual targets
              VisionConstants.poseStrategy));
    }

    return Optional.empty();
  }

  /**
   * Get the best estimated robot pose from all cameras
   *
   * @return Optional containing the best estimated robot pose, or empty if no valid estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
    // This will return the best pose that was calculated during the last periodic
    // update
    if (!hasTarget) {
      return Optional.empty();
    }

    // Find the camera with a valid pose
    for (Pose3d cameraPose : cameraPoses) {
      if (cameraPose != Pose3d.kZero) {
        // Construct an estimated pose with this camera's data
        return Optional.of(
            new EstimatedRobotPose(
                cameraPose,
                Timer.getFPGATimestamp(),
                new ArrayList<>(), // Ideally, store the actual targets
                VisionConstants.poseStrategy));
      }
    }

    return Optional.empty();
  }

  /**
   * Calculate the average distance to a list of targets
   *
   * @param targets List of tracked targets
   * @return Average distance in meters
   */
  private double calculateAverageDistance(List<PhotonTrackedTarget> targets) {
    if (targets.isEmpty()) {
      return 0.0;
    }

    double totalDistance = 0.0;
    for (PhotonTrackedTarget target : targets) {
      totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
    }

    return totalDistance / targets.size();
  }

  /**
   * Finds the closest reef tag to the robot's current position.
   *
   * @param robotPose The current estimated robot pose
   * @return The pose of the closest reef tag
   */
  public Pose2d findClosestReefTag(Pose2d robotPose) {
    return robotPose.nearest(reefTagPoses);
  }

  /**
   * Get the standard deviation matrix for the current vision estimate
   *
   * @return 3x1 matrix with standard deviations for x, y, and theta
   */
  public Matrix<N3, N1> getEstimationStdDev() {
    return stdDev;
  }

  /**
   * Check if the vision system currently has a valid target
   *
   * @return True if at least one valid target is visible
   */
  public boolean hasTarget() {
    return hasTarget;
  }

  /**
   * Get the list of all reef tag poses on the field
   *
   * @return List of Pose2d objects for reef tags
   */
  public List<Pose2d> getReefTagPoses() {
    return new ArrayList<>(reefTagPoses);
  }

  // Helper class for camera configuration
  private static class CameraConfig {
    public final String name;
    public final Transform3d transform;

    public CameraConfig(String name, Transform3d transform) {
      this.name = name;
      this.transform = transform;
    }
  }
}
