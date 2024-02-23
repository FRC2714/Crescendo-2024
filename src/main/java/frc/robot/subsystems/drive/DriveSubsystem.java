// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.Vision;
import frc.robot.utils.FieldRelativeAcceleration;
import frc.robot.utils.FieldRelativeVelocity;
import frc.robot.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private FieldRelativeVelocity m_fieldRelativeVelocity = new FieldRelativeVelocity();
  private FieldRelativeVelocity m_lastFieldRelativeVelocity = new FieldRelativeVelocity();
  private FieldRelativeAcceleration m_fieldRelativeAcceleration = new FieldRelativeAcceleration();

  private Vision m_backPhotonCamera = new Vision(PhotonConstants.kBackCameraName, PhotonConstants.kBackCameraLocation);
  private Vision m_frontPhotonCamera = new Vision(PhotonConstants.kFrontCameraName, PhotonConstants.kFrontCameraLocation);

  private Field2d m_field = new Field2d();
  
  // Pose class for tracking robot pose
  Vector<N3> stateStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.01)); // Increase for less state trust
  Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1)); // Increase for less vision trust

  SwerveDrivePoseEstimator m_pose = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      new Rotation2d(m_gyro.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }, DriveConstants.kInitialRedPose, stateStdDevs, visionMeasurementStdDevs);
  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_gyro.calibrate();
  }
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    SmartDashboard.putNumber("Angle to goal", Units.radiansToDegrees(getRotationFromGoalRadians(getPose())));
    
    m_pose.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    Optional<EstimatedRobotPose> backPhotonPoseEstimation = m_backPhotonCamera.getEstimatedGlobalPose();
    Optional<EstimatedRobotPose> frontPhotonPoseEstimation = m_frontPhotonCamera.getEstimatedGlobalPose();
    frontPhotonPoseEstimation.ifPresentOrElse(poseEstimation -> {
      Pose2d estPose = poseEstimation.estimatedPose.toPose2d();
      SmartDashboard.putNumber("drive photon est x", poseEstimation.estimatedPose.getX());
      SmartDashboard.putNumber("drive photon est y", poseEstimation.estimatedPose.getY());
      SmartDashboard.putNumber("drive photon est theta", poseEstimation.estimatedPose.getRotation().toRotation2d().getDegrees());
      m_pose.addVisionMeasurement(estPose, poseEstimation.timestampSeconds);
    }, new Runnable() {
        @Override
        public void run() {
          // backPhotonPoseEstimation.ifPresent(poseEstimation -> {
          //   Pose2d estPose = poseEstimation.estimatedPose.toPose2d();
          //   SmartDashboard.putNumber("drive photon est x", poseEstimation.estimatedPose.getX());
          //   SmartDashboard.putNumber("drive photon est y", poseEstimation.estimatedPose.getY());
          //   m_pose.addVisionMeasurement(estPose, poseEstimation.timestampSeconds, m_backPhotonCamera.getEstimationStdDevs(estPose));
          // });
        }
    });

    
    m_fieldRelativeVelocity = new FieldRelativeVelocity(getChassisSpeed(), new Rotation2d(m_gyro.getAngle(IMUAxis.kZ)));
    m_fieldRelativeAcceleration = new FieldRelativeAcceleration(m_fieldRelativeVelocity, m_lastFieldRelativeVelocity, 0.02);
    m_lastFieldRelativeVelocity = m_fieldRelativeVelocity;

    m_field.setRobotPose(getPose());

    SmartDashboard.putData("Field Position", m_field);
    SmartDashboard.putNumber("Distance to goal meters", getDistanceToGoalMeters(getPose()));
    SmartDashboard.putNumber("Pose rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Pose rotation to goal", Units.radiansToDegrees(getRotationFromGoalRadians(getPose())));
  }

  public double getSpeakerTargetYaw() {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get().toString().equals("Red")) {
        for (PhotonTrackedTarget i : m_frontPhotonCamera.getLatestResult().getTargets()) {
          if (i.getFiducialId() == 4) {
            return i.getBestCameraToTarget().plus(
              new Transform3d(0, -12, 0, new Rotation3d(0, 0, 0))).getRotation().getZ();
          }
        }
      }
      else {
        for (PhotonTrackedTarget i : m_frontPhotonCamera.getLatestResult().getTargets()) {
            if (i.getFiducialId() == 7) {
              return i.getBestCameraToTarget().plus(
                new Transform3d(0, -12, 0, new Rotation3d(0, 0, 0))).getRotation().getZ();
            }
        }
      }
    }
    return 0;
  }

  public double getDistanceToGoalMeters(Pose2d pose) {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get().toString().equals("Red"))
        return Math.sqrt(
          Math.pow(Math.abs(pose.getX() - FieldConstants.kRedSpeakerAprilTagLocation.getX()), 2) + 
          Math.pow(Math.abs(pose.getY() - FieldConstants.kRedSpeakerAprilTagLocation.getY()), 2)
        );
    else
      return Math.sqrt(
          Math.pow(Math.abs(pose.getX() - FieldConstants.kBlueSpeakerAprilTagLocation.getX()), 2) + 
          Math.pow(Math.abs(pose.getY() - FieldConstants.kBlueSpeakerAprilTagLocation.getY()), 2)
        );
    }
    return Math.sqrt(
        Math.pow(Math.abs(pose.getX() - FieldConstants.kRedSpeakerAprilTagLocation.getX()), 2) + 
        Math.pow(Math.abs(pose.getY() - FieldConstants.kRedSpeakerAprilTagLocation.getY()), 2)
      );
  }

  public double getRotationFromGoalRadians(Pose2d pose) {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get().toString().equals("Red"))
        return Math.atan(
          (pose.getY() - FieldConstants.kRedSpeakerAprilTagLocation.getY()) /
          Math.abs(pose.getX() - FieldConstants.kRedSpeakerAprilTagLocation.getX())
        );
    else
      return Math.PI - Math.atan(
          (pose.getY() - FieldConstants.kBlueSpeakerAprilTagLocation.getY()) /
          Math.abs(pose.getX() - FieldConstants.kBlueSpeakerAprilTagLocation.getX())
        );
    }
    return Math.atan(
          (pose.getY() - FieldConstants.kRedSpeakerAprilTagLocation.getY())  /
          Math.abs(pose.getX() - FieldConstants.kRedSpeakerAprilTagLocation.getX())
        );
  }

  public Rotation2d getYawToPose() {
    if (DriverStation.getAlliance().isPresent())
      if (DriverStation.getAlliance().get().toString().equals("Red"))
        return PhotonUtils.getYawToPose(getPose(), FieldConstants.kRedSpeakerAprilTagLocation);
    return PhotonUtils.getYawToPose(getPose(), FieldConstants.kBlueSpeakerAprilTagLocation);
  }

  public FieldRelativeVelocity getFieldRelativeVelocity() {
    return m_fieldRelativeVelocity;
  }

  public FieldRelativeAcceleration getFieldRelativeAcceleration() {
    return m_fieldRelativeAcceleration;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_pose.getEstimatedPosition();
  }

  /**
   * Resets the pose to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPoseEstimator(Pose2d pose) {
    m_pose.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Converts the 4 swerve module states into a chassisSpeed by making use of the
   * swerve drive kinematics.
   * 
   * @return ChassisSpeeds object containing robot X, Y, and Angular velocity
   */
  public ChassisSpeeds getChassisSpeed() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    resetPoseEstimator(new Pose2d());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
