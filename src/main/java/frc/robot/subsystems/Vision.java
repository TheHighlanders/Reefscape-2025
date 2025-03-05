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
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

final class VisionConstants {
  static final Transform3d frontFacingCam = new Transform3d(
      new Translation3d(0.205486, -0.122174, 0.4376928),
      new Rotation3d(0, Units.degreesToRadians(25.5), 0));

  static final int[] reefTagIds = {
      6, 7, 8, 9, 10, 11, // RED
      17, 18, 19, 20, 21, 22 // BLUE
  };

  // static final Transform3d robotRightCamPosition = new Transform3d(
  // new Translation3d(0.233, -0.288, 16.965),
  // new Rotation3d(0, -Units.degreesToRadians(25), -Units.degreesToRadians(25)));

  static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.023, 0.026, 0.1382300768); // TODO: BS
  static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 0.1); // TODO: BS
}

public class Vision extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("ReefCamera");
  PhotonPoseEstimator poseEst;
  AprilTagFieldLayout aprilTagFieldLayout;

  private Matrix<N3, N1> stdDev = new Matrix<N3, N1>(Nat.N3(), Nat.N1());

  StructPublisher<Pose2d> visionEstPose = NetworkTableInstance.getDefault()
      .getStructTopic("/Vision/Vision Estimated Pose", Pose2d.struct).publish();

  List<Pose2d> reefTagPoses = new ArrayList<>();


  /** Creates a new Vision. */
  public Vision() {
    // this.aprilTagFieldLayout =
    // AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    poseEst = new PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        VisionConstants.frontFacingCam);
    poseEst.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


    for (int tagId : VisionConstants.reefTagIds) {
      var tagPoseOptional = aprilTagFieldLayout.getTagPose(tagId);
      if (tagPoseOptional.isPresent()) {
        reefTagPoses.add(tagPoseOptional.get().toPose2d());
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Optional<EstimatedRobotPose> estPose = getEstimatedRobotPose();

    if (estPose.isPresent()) {
      stdDev = updateEstimationStdDevs(estPose, estPose.get().targetsUsed, poseEst);
      visionEstPose.set(estPose.get().estimatedPose.toPose2d());
    }
  }

  public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
    Optional<PhotonPipelineResult> result;

    var camResult = camera.getAllUnreadResults();
    if (camResult.size() != 0) {
      result = Optional.of(camResult.get(camResult.size() - 1));
    } else {
      result = Optional.empty();
    }

    Optional<EstimatedRobotPose> estPose;
    if (result.isPresent()) {
      boolean resultAmbiguous = false;
      double ambiguityRatio = -666;
      for(PhotonTrackedTarget target : result.get().getTargets()){
        ambiguityRatio = target.poseAmbiguity;
        if(target.poseAmbiguity > 0.001){
          resultAmbiguous = true;
        }
      }
      SmartDashboard.putBoolean("Vision/Ambiguity>0.2", resultAmbiguous);

      SmartDashboard.putNumber("Vision/Ambiguity", ambiguityRatio);

      if(!resultAmbiguous){
        estPose = poseEst.update(result.get());
      } else {estPose = Optional.empty();}

    } else {
      estPose = Optional.empty();
    }

    return estPose;
    // return Optional.empty();
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
        curStdDevs = VisionConstants.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = VisionConstants.kMultiTagStdDevs;
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

  /**
   * Finds the closest reef tag to the robot's current position.
   *
   * @param robotPose The current estimated robot pose
   * @return Optional containing the pose of the closest reef tag, or empty if
   *         none found
   */
  public Pose2d findClosestReefTag(Pose2d robotPose) {
    // Get poses 



    return robotPose.nearest(reefTagPoses);
  }

  public Matrix<N3, N1> getEstimationStdDev() {
    return stdDev;
  }
}
