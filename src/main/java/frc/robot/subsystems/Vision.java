// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  List<PhotonCamera> cameras;
  List<PhotonPoseEstimator> photonPoseEstimators;

  private List<Matrix<N3, N1>> allStdDevs;

  private Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  private Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  Transform3d robotToLeftCam = new Transform3d(new Translation3d(0.233, -0.288, 0.259),
      new Rotation3d(0, -Units.degreesToRadians(25), -Units.degreesToRadians(25)));
  Transform3d robotToFrontCam = new Transform3d(new Translation3d(0.233, -0.288, 0.259),
      new Rotation3d(0, -Units.degreesToRadians(25), -Units.degreesToRadians(25)));

  /** Creates a new Vision. */
  public Vision() {
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    cameras.add(new PhotonCamera("leftCamera"));
    cameras.add(new PhotonCamera("frontCamera"));

    photonPoseEstimators
        .add(new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToLeftCam));
    photonPoseEstimators
        .add(new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToFrontCam));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var estPoses = getEstimatedRobotPoses();
    for (int i = 0; i < estPoses.size(); i++) {
      Optional<EstimatedRobotPose> estPose = estPoses.get(i);
      if (estPose.isPresent()) {
        allStdDevs.set(i,updateEstimationStdDevs(estPose, estPose.get().targetsUsed, photonPoseEstimators.get(i)));
      }
    }
  }

  public List<Optional<EstimatedRobotPose>> getEstimatedRobotPoses() {
    List<Optional<PhotonPipelineResult>> results = new ArrayList<Optional<PhotonPipelineResult>>();

    for (PhotonCamera camera : cameras) {
      var camResults = camera.getAllUnreadResults();
      if (camResults.size() >= 1) {
        results.add(Optional.of(camResults.get(camResults.size() - 1)));
      } else {
        results.add(Optional.empty());
      }
    }

    List<Optional<EstimatedRobotPose>> estPoses = new ArrayList<Optional<EstimatedRobotPose>>();
    // Build array of est poses from each camera

    for (int i = 0; i < results.size(); i++) {
      var result = results.get(i);

      if (result.isPresent()) {
        estPoses.add(photonPoseEstimators.get(i).update(result.get()));
      } else {
        estPoses.add(Optional.empty());
      }
    }

    return estPoses;
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates
   * dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from
   * the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets       All targets in this camera frame
   */
  private Matrix<N3,N1> updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator poseEst) {
      Matrix<N3,N1> curStdDevs;
    
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = poseEst.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }

    return curStdDevs;
  }

  public List<Matrix<N3, N1>> getEstimationStdDevs() {
    return allStdDevs;
  }

}
