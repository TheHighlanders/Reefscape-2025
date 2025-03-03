// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

final class VisionConstants {
  static final Transform3d frontFacingCam =
      new Transform3d(
          new Translation3d(0.205486, -0.122174, 0.4376928),
          new Rotation3d(0, Units.degreesToRadians(25.5), 0));

  public static final int[] reefTagIds = {1, 2, 3, 4}; // TODO: make real tags

  // static final Transform3d robotRightCamPosition = new Transform3d(
  // new Translation3d(0.233, -0.288, 16.965),
  // new Rotation3d(0, -Units.degreesToRadians(25), -Units.degreesToRadians(25)));

  static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, 0.5); // TODO: BS
  static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1); // TODO: BS
}

public class Vision extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("ReefCamera");
  PhotonPoseEstimator poseEst;
  AprilTagFieldLayout aprilTagFieldLayout;

  private Matrix<N3, N1> stdDev;

  /** Creates a new Vision. */
  public Vision() {
    // this.aprilTagFieldLayout =
    // AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    poseEst =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.frontFacingCam);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Optional<EstimatedRobotPose> estPose = getEstimatedRobotPose();

    if (estPose.isPresent()) {
      stdDev = updateEstimationStdDevs(estPose, estPose.get().targetsUsed, poseEst);
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
    Optional<PhotonPipelineResult> result;

    var camResult = camera.getAllUnreadResults();
    if (camResult.size() != 0) {
      result = Optional.of(camResult.get(0));
    } else {
      result = Optional.empty();
    }

    Optional<EstimatedRobotPose> estPose;
    if (result.isPresent()) {
      estPose = poseEst.update(result.get());
    } else {
      estPose = Optional.empty();
    }

    return estPose;
    // return Optional.empty();
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private Matrix<N3, N1> updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose,
      List<PhotonTrackedTarget> targets,
      PhotonPoseEstimator poseEst) {
    Matrix<N3, N1> curStdDevs;

    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = VisionConstants.kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = VisionConstants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = poseEst.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = VisionConstants.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }

    return curStdDevs;
  }

  /**
   * Finds the closest reef tag to the robot's current position.
   *
   * @param robotPose The current estimated robot pose
   * @return Optional containing the pose of the closest reef tag, or empty if none found
   */
  public Optional<Pose2d> findClosestReefTag(Pose2d robotPose) {
    List<Integer> visibleReefTagIds = new ArrayList<>();

    Optional<EstimatedRobotPose> estPose = getEstimatedRobotPose();

    // Find all visible reef tag IDs
    if (estPose.isPresent()) {
      for (PhotonTrackedTarget target : estPose.get().targetsUsed) {
        int tagId = target.getFiducialId();
        for (int reefId : VisionConstants.reefTagIds) {
          if (tagId == reefId && !visibleReefTagIds.contains(tagId)) {
            visibleReefTagIds.add(tagId);
          }
        }
      }
    }

    // Get poses for all visible reef tags from the field layout
    List<Pose2d> reefTagPoses = new ArrayList<>();

    for (int tagId : visibleReefTagIds) {
      var tagPoseOptional = aprilTagFieldLayout.getTagPose(tagId);
      if (tagPoseOptional.isPresent()) {
        reefTagPoses.add(tagPoseOptional.get().toPose2d());
      }
    }

    // Find the closest reef tag
    if (!reefTagPoses.isEmpty()) {
      return reefTagPoses.stream()
          .min(
              Comparator.comparingDouble(
                  pose -> pose.getTranslation().getDistance(robotPose.getTranslation())));
    }

    return Optional.empty();
  }

  public Matrix<N3, N1> getEstimationStdDev() {
    return stdDev;
  }
}
