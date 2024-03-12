// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
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

  private PhotonCamera photonCamera;
  private PhotonPoseEstimator photonPoseEstimator;

  public Vision(String cameraName, Transform3d cameraLocation) {
    photonCamera = new PhotonCamera(cameraName);
    // photonCamera.setPipelineIndex(0);
    photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.kAprilTagFieldLayout,
                                                  PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                                  photonCamera,
                                                  cameraLocation);
    // photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public PhotonPipelineResult getLatestResult() {
    return photonCamera.getLatestResult();
  }

  public double getXOffsetDegrees() {
    for (PhotonTrackedTarget i : getLatestResult().getTargets()) {
      if (i.getFiducialId() == 4 && DriverStation.getAlliance().get().toString().equals("Red")) {
        return i.getYaw();
      }
      else if (i.getFiducialId() == 7 && DriverStation.getAlliance().get().toString().equals("Blue")) {
        return i.getYaw();
      }
    }
    return 0;
  }

  public boolean speakerVisible() {
    for (PhotonTrackedTarget i : getLatestResult().getTargets()) {
      if (i.getFiducialId() == 4) {
        return true;
      }
      else if (i.getFiducialId() == 7) {
        return true;
      }
    }
    return false;
  }

  public Transform3d getSpeakerTargetTransform() {
    if (getSpeakerTarget() == null) return null;
    return getSpeakerTarget().getBestCameraToTarget().plus(PhotonConstants.kFrontCameraLocation.inverse());
  }

  public PhotonTrackedTarget getSpeakerTarget() { //add camera offset
    for (PhotonTrackedTarget i : getLatestResult().getTargets()) {
      if (i.getFiducialId() == 4 ) {
        return i;
      }
      else if (i.getFiducialId() == 7) {
        return i;
      }
    }
    return null;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      return photonPoseEstimator.update();
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

    public double getHeadingRelativeAprilTagDegrees(double gyroHeading)
    {
      if(getSpeakerTarget() != null)
      {
        return gyroHeading - Math.toDegrees(getSpeakerTarget().getYaw());
      }
        return gyroHeading;
    }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Optional<EstimatedRobotPose> photonPoseEstimation = getEstimatedGlobalPose();
    photonPoseEstimation.ifPresent(poseEstimation -> {
      SmartDashboard.putNumber("pv X", poseEstimation.estimatedPose.getX());
      SmartDashboard.putNumber("pv Y", poseEstimation.estimatedPose.getY());
    });

    SmartDashboard.putBoolean("Speaker?", speakerVisible());

    SmartDashboard.putNumber("offset deg", getXOffsetDegrees());
    // SmartDashboard.putNumber("Best target x distance", getMultiTagLatestResult().estimatedPose.best.getX());
    // SmartDashboard.putNumber("Best target y distance", getMultiTagLatestResult().estimatedPose.best.getY());
    SmartDashboard.putBoolean("photon pose", photonPoseEstimation.isPresent());
    //SmartDashboard.putNumber("april tag heading degrees", getSpeakerTarget().getYaw());

    // SmartDashboard.putNumber("Number of targets", photonCamera.getLatestResult().getTargets().size());
    // SmartDashboard.putNumber("Best Target ID", photonCamera.getLatestResult().getBestTarget().getFiducialId());
    // SmartDashboard.putNumber("Best Target Distance", photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget().getX());
    // System.out.println("Targets: "+ photonCamera.getLatestResult().getTargets());
    // System.out.println("Best target: "+ photonCamera.getLatestResult().getBestTarget());

    if (getLatestResult().hasTargets()){
      // List<PhotonTrackedTarget> targets = photonCamera.getLatestResult().getTargets();
      // SmartDashboard.putNumber("number of targets", targets.size());
      // for(PhotonTrackedTarget i : targets)
      // {
      //   if(i.getFiducialId() == 7)
      //   {
      //     SmartDashboard.putBoolean("got 7", true);
      //     SmartDashboard.putNumber("speakerYaw", i.getYaw());
      //     SmartDashboard.putNumber("speakerDistance", i.getBestCameraToTarget().getX());
      //   }
      // }
      PhotonTrackedTarget speakerTarget = getSpeakerTarget();
      if(speakerTarget != null){
        SmartDashboard.putNumber("speakerYaw", Math.toDegrees(speakerTarget.getYaw()));
        SmartDashboard.putNumber("speakerDistance", speakerTarget.getBestCameraToTarget().getX());
        SmartDashboard.putNumber("speakerZ", Math.toDegrees(speakerTarget.getBestCameraToTarget().getRotation().getZ()));
        double angleToSpeaker =  -speakerTarget.getBestCameraToTarget().getRotation().getZ();
        double angleToTurn = angleToSpeaker > 0 ? angleToSpeaker: (Math.PI * 2) + angleToSpeaker;
        SmartDashboard.putNumber("angleTurn", Math.toDegrees(angleToTurn));
  

    // m_drivetrain.drive(
    //   0, 
    //   0, 
    //   thetaController.calculate(m_camera.getSpeakerTarget().getYaw()),
    //   true,
    //   false);
        SmartDashboard.putBoolean("noSpeaker", false);
    } else {
        SmartDashboard.putBoolean("noSpeaker", true);

    } 
  } else {
        SmartDashboard.putBoolean("noTarget", true);

    }
}
}
