// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonConstants;
import frc.robot.Constants.FieldConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private double timeNoTargetSeen = 0;

  private PhotonCamera photonCamera;
  private PhotonPoseEstimator photonPoseEstimator;

  private double currentDistance, currentRotation;

  public Vision(String cameraName, Transform3d cameraLocation) {
    photonCamera = new PhotonCamera(cameraName);
    // photonCamera.setPipelineIndex(0);
    photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.kAprilTagFieldLayout,
                                                  PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                                  photonCamera,
                                                  cameraLocation);
    // photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    currentDistance = 0;
    currentRotation = 0;
  }

  public PhotonPipelineResult getLatestResult() {
    return photonCamera.getLatestResult();
  }

  public double getXOffsetDegrees() {
    for (PhotonTrackedTarget i : getLatestResult().getTargets()) {
      if (i.getFiducialId() == 4 && DriverStation.getAlliance().get().toString().equals("Red")) {
        currentRotation = i.getYaw() - 3;
      }
      else if (i.getFiducialId() == 7 && DriverStation.getAlliance().get().toString().equals("Blue")) {
        currentRotation = i.getYaw() -+ 3;
      }
    }
    return currentRotation;
  }

  public double getDistanceToGoalMeters() {
    for (PhotonTrackedTarget i : getLatestResult().getTargets()) {
      if (i.getFiducialId() == 4 && DriverStation.getAlliance().get().toString().equals("Red")) {
        currentDistance = Math.sqrt(Math.pow(i.getBestCameraToTarget().getX(), 2) + Math.pow(i.getBestCameraToTarget().getY(), 2));
      }
      else if (i.getFiducialId() == 7 && DriverStation.getAlliance().get().toString().equals("Blue")) {
        currentDistance = Math.sqrt(Math.pow(i.getBestCameraToTarget().getX(), 2) + Math.pow(i.getBestCameraToTarget().getY(), 2));
      }
    }
    return currentDistance;
  }

  public boolean speakerVisible() {
    for (PhotonTrackedTarget i : getLatestResult().getTargets()) {
      if (i.getFiducialId() == 4 && DriverStation.getAlliance().get().toString().equals("Red")) {
        return true;
      }
      else if (i.getFiducialId() == 7 && DriverStation.getAlliance().get().toString().equals("Blue")) {
        return true;
      }
    }
    return false;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      return photonPoseEstimator.update();
  }

  public boolean hasTargets() {
    return timeNoTargetSeen < 100;
  }

  public PhotonTrackedTarget getBestTarget() {
    return getLatestResult().getBestTarget();
  }

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = PhotonConstants.kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = PhotonConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Optional<EstimatedRobotPose> photonPoseEstimation = getEstimatedGlobalPose();
    photonPoseEstimation.ifPresent(poseEstimation -> {
      SmartDashboard.putNumber("pv X", poseEstimation.estimatedPose.getX());
      SmartDashboard.putNumber("pv Y", poseEstimation.estimatedPose.getY());
    });

    if (!getLatestResult().hasTargets()) {
      timeNoTargetSeen += 20;
    }
    else {
      timeNoTargetSeen = 0;
    }

    SmartDashboard.putNumber("Drive offset", getXOffsetDegrees());

    SmartDashboard.putNumber("Distance To GOal Meters", getDistanceToGoalMeters());

    SmartDashboard.putBoolean("Speaker?", speakerVisible());

    SmartDashboard.putNumber("offset deg", getXOffsetDegrees());
    // SmartDashboard.putNumber("Best target x distance", getMultiTagLatestResult().estimatedPose.best.getX());
    // SmartDashboard.putNumber("Best target y distance", getMultiTagLatestResult().estimatedPose.best.getY());
    SmartDashboard.putBoolean("photon pose", photonPoseEstimation.isPresent());
    // SmartDashboard.putNumber("Number of targets", photonCamera.getLatestResult().getTargets().size());
    // SmartDashboard.putNumber("Best Target ID", photonCamera.getLatestResult().getBestTarget().getFiducialId());
    // SmartDashboard.putNumber("Best Target Distance", photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget().getX());
    // System.out.println("Targets: "+ photonCamera.getLatestResult().getTargets());
    // System.out.println("Best target: "+ photonCamera.getLatestResult().getBestTarget());
  }
}
