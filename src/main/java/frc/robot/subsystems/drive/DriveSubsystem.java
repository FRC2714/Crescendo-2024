// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.ThetaPIDConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PeriodicConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Vision;
import frc.robot.utils.FieldRelativeAcceleration;
import frc.robot.utils.FieldRelativeVelocity;
import frc.robot.utils.SwerveUtils;
import frc.robot.utils.TunableNumber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  private boolean rotatingToGoal = false;
  private boolean stopped = false;

  private FieldRelativeVelocity m_fieldRelativeVelocity = new FieldRelativeVelocity();
  private FieldRelativeVelocity m_lastFieldRelativeVelocity = new FieldRelativeVelocity();
  private FieldRelativeAcceleration m_fieldRelativeAcceleration = new FieldRelativeAcceleration();

  // private Vision m_backPhotonCamera = new Vision(PhotonConstants.kBackCameraName, PhotonConstants.kBackCameraLocation);
  // private Vision m_frontPhotonCamera = new Vision(PhotonConstants.kFrontCameraName, PhotonConstants.kFrontCameraLocation);

  private Field2d m_field = new Field2d();

  private Vision m_camera;

  private TunableNumber translationP, rotationP;

  private double maxSpeedMetersPerSecond, maxAngularSpeed;
  
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
      }, DriverStation.getAlliance().toString().equals("Red")
        ? DriveConstants.kInitialRedPose
        : DriveConstants.kInitialBluePose,
        stateStdDevs,
        visionMeasurementStdDevs);
  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(Vision m_camera) {
    m_gyro.calibrate();
    this.m_camera = m_camera;

    maxSpeedMetersPerSecond = DriveConstants.kAutoMaxSpeedMetersPerSecond;
    maxAngularSpeed = DriveConstants.kAutoMaxAngularSpeed;
    
    PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    translationP = new TunableNumber("translation P");
    rotationP = new TunableNumber("rotation P");
    
    translationP.setDefault(1.5);
    rotationP.setDefault(2.8);

// PPHolonomicDriveController.setRotationTargetOverride(this::getDriveRotationToGoalOptional);
    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPoseEstimator, // Method to reset odometry (will be called if your auto has a starti ng pose)
            this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(1.5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(1.5, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.3706, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block

    SmartDashboard.putNumber("rotation max", maxAngularSpeed);

    if (translationP.hasChanged() || rotationP.hasChanged()) {
      AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPoseEstimator, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(translationP.get(), 0.0, 0.0), // Translation PID constants
                    new PIDConstants(rotationP.get(), 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.3706, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    }

    SmartDashboard.putBoolean("Stopped?", stopped);

    SmartDashboard.putNumber("turn rate", getTurnRate());

    SmartDashboard.putNumber("front left velocity", Math.abs(m_frontLeft.getState().speedMetersPerSecond));
    SmartDashboard.putNumber("front right velocity", Math.abs(m_frontRight.getState().speedMetersPerSecond));
    SmartDashboard.putNumber("back left velocity", Math.abs(m_rearLeft.getState().speedMetersPerSecond));
    SmartDashboard.putNumber("back right velocity", Math.abs(m_rearRight.getState().speedMetersPerSecond));

    SmartDashboard.putNumber("front left setpoint", Math.abs(m_frontLeft.getDesiredStateSpeed()));
    SmartDashboard.putNumber("front right setpoint", Math.abs(m_frontRight.getDesiredStateSpeed()));
    SmartDashboard.putNumber("back left setpoint", Math.abs(m_rearLeft.getDesiredStateSpeed()));
    SmartDashboard.putNumber("back right setpoint", Math.abs(m_rearRight.getDesiredStateSpeed()));

    // SmartDashboard.putNumber("Angle to goal", Units.radiansToDegrees(getSpeakerTargetYaw()));
    
    m_pose.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    // Optional<EstimatedRobotPose> backPhotonPoseEstimation = m_backPhotonCamera.getEstimatedGlobalPose();
    // Optional<EstimatedRobotPose> frontPhotonPoseEstimation = m_frontPhotonCamera.getEstimatedGlobalPose();
    // frontPhotonPoseEstimation.ifPresentOrElse(poseEstimation -> {
    //   Pose2d estPose = poseEstimation.estimatedPose.toPose2d();
    //   SmartDashboard.putNumber("drive photon est x", poseEstimation.estimatedPose.getX());
    //   SmartDashboard.putNumber("drive photon est y", poseEstimation.estimatedPose.getY());
    //   SmartDashboard.putNumber("drive photon est theta", poseEstimation.estimatedPose.getRotation().toRotation2d().getDegrees());
    //   m_pose.addVisionMeasurement(estPose, poseEstimation.timestampSeconds);
    // }, new Runnable() {
        // @Override
        // public void run() {
          // backPhotonPoseEstimation.ifPresent(poseEstimation -> {
          //   Pose2d estPose = poseEstimation.estimatedPose.toPose2d();
          //   SmartDashboard.putNumber("drive photon est x", poseEstimation.estimatedPose.getX());
          //   SmartDashboard.putNumber("drive photon est y", poseEstimation.estimatedPose.getY());
          //   m_pose.addVisionMeasurement(estPose, poseEstimation.timestampSeconds, m_backPhotonCamera.getEstimationStdDevs(estPose));
          // });
        // }
    // });

    
    m_fieldRelativeVelocity = new FieldRelativeVelocity(getChassisSpeed(), new Rotation2d(m_gyro.getAngle(IMUAxis.kZ)));
    m_fieldRelativeAcceleration = new FieldRelativeAcceleration(m_fieldRelativeVelocity, m_lastFieldRelativeVelocity, 0.02);
    m_lastFieldRelativeVelocity = m_fieldRelativeVelocity;

    m_field.setRobotPose(getPose());

    SmartDashboard.putNumber("Gyro Angle", (m_gyro.getAngle(IMUAxis.kZ) % 360));

    SmartDashboard.putNumber("gyro heading", getHeading());
    SmartDashboard.putData("Field Position", m_field);
    SmartDashboard.putNumber("Pose X", getPose().getX());
    SmartDashboard.putNumber("Distance to goal meters", getDistanceToGoalMeters(getPose()));
    SmartDashboard.putNumber("Pose rotation", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Pose rotation to goal", Units.radiansToDegrees(getRotationFromGoalRadians(getPose())));
  }

  public Command setMaxSpeedMetersPerSecond(double maxSpeedMetersPerSecond) {
    return new InstantCommand(() -> this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond);
  }

  public Command setMaxAngularSpeed(double maxAngularSpeed) {
    return new InstantCommand(() -> this.maxAngularSpeed = maxAngularSpeed);
  }

  // public double getSpeakerTargetYaw() {
  //   if (DriverStation.getAlliance().isPresent()) {
  //     if (DriverStation.getAlliance().get().toString().equals("Red")) {
  //       for (PhotonTrackedTarget i : m_frontPhotonCamera.getLatestResult().getTargets()) {
  //         if (i.getFiducialId() == 4) {
  //           return i.getBestCameraToTarget().plus(
  //             new Transform3d(0, -12, 0, new Rotation3d(0, 0, 0))).getRotation().getZ();
  //         }
  //       }
  //     }
  //     else {
  //       for (PhotonTrackedTarget i : m_frontPhotonCamera.getLatestResult().getTargets()) {
  //           if (i.getFiducialId() == 7) {
  //             return i.getBestCameraToTarget().plus(
  //               new Transform3d(0, -12, 0, new Rotation3d(0, 0, 0))).getRotation().getZ();
  //           }
  //       }
  //     }
  //   }
  //   return 0;
  // }

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
  public Optional<Rotation2d> getRotationTargetOverride(){
    // Some condition that should decide if we want to override rotation
    if(rotatingToGoal) {
        // Return an optional containing the rotation override (this should be a field relative rotation)
        return Optional.of(Rotation2d.fromDegrees(getDriveRotationToGoal()));
    } else {
        // return an empty optional when we don't want to override the path's rotation
        return Optional.empty();
    }
  }

  public Command enableVoltageCompensation() {
    return new ParallelCommandGroup(
      m_frontLeft.enableModuleVoltageCompensation(),
      m_frontRight.enableModuleVoltageCompensation(),
      m_rearLeft.enableModuleVoltageCompensation(),
      m_rearRight.enableModuleVoltageCompensation()
    );
  }

  public Command disableVoltageCompensation() {
    return new ParallelCommandGroup(
      m_frontLeft.disableModuleVoltageCompensation(),
      m_frontRight.disableModuleVoltageCompensation(),
      m_rearLeft.disableModuleVoltageCompensation(),
      m_rearRight.disableModuleVoltageCompensation()
    );
  }

  public void toggleRotatingToGoal() {
    rotatingToGoal = !rotatingToGoal;
  }

  public void setRotatingToGoal() {
    rotatingToGoal = true;
  }

  public void disableRotatingToGoal() {
    rotatingToGoal = false;
  }

  public boolean getRotatingToGoal(double joystickInput) {
    if (Math.abs(joystickInput) > 0) {
      setMaxAngularSpeed(DriveConstants.kTeleOpMaxAngularSpeed).schedule();
      rotatingToGoal = false;
    }
    return rotatingToGoal;
  }

  public Command toggleRotatingToGoalCommand() {
    return new InstantCommand(() -> toggleRotatingToGoal());
  }

  public Command setRotatingToGoalCommand() {
    return new SequentialCommandGroup(setMaxAngularSpeed(DriveConstants.kAutoRotatingMaxAngularSpeed),
    new InstantCommand(() -> setRotatingToGoal()));
  }

  public Command disableRotatingToGoalCommand() {
    return new InstantCommand(() -> disableRotatingToGoal());
  }

  public double getDriveRotationToGoalPose() {
    PIDController thetaController = new PIDController(ThetaPIDConstants.kP, ThetaPIDConstants.kI, ThetaPIDConstants.kD);
    thetaController.setTolerance(Units.degreesToRadians(0),0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    double rotationToGoal = getRotationFromGoalRadians(getPose());
    thetaController.setSetpoint(-rotationToGoal);

    return thetaController.calculate(getPose().getRotation().getRadians());
  }

  public double getDriveRotationToGoal() {
    PIDController thetaController = new PIDController(ThetaPIDConstants.kP, ThetaPIDConstants.kI, ThetaPIDConstants.kD);
    thetaController.setTolerance(Units.degreesToRadians(0),0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    thetaController.setSetpoint(Units.degreesToRadians(m_camera.getSpeakerXOffsetDegrees()) < 0
      ? Units.degreesToRadians(-15) / m_camera.getDistanceToGoalMeters()
      : Units.degreesToRadians(15) / m_camera.getDistanceToGoalMeters());

    return m_camera.speakerVisible() ? thetaController.calculate(Units.degreesToRadians(m_camera.getSpeakerXOffsetDegrees())) : 0;
  }

  public Command enableStopped() {
    return new InstantCommand(() -> stopped = true);
  }

  public Command disableStopped() {
    return new InstantCommand(() -> stopped = false);
  }

  // public Optional<Rotation2d> getDriveRotationToGoalOptional() {
  //   PIDController thetaController = new PIDController(ThetaPIDConstants.kP, ThetaPIDConstants.kI, ThetaPIDConstants.kD);
  //   thetaController.setTolerance(Units.degreesToRadians(0),0);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   thetaController.setSetpoint(Units.degreesToRadians(m_camera.getSpeakerXOffsetDegrees()) < 0 ? Units.degreesToRadians(-15) / m_camera.getDistanceToGoalMeters() : Units.degreesToRadians(15) / m_camera.getDistanceToGoalMeters());
  //   return (m_camera.speakerVisible() && stopped) ? Optional.of(new Rotation2d(Units.degreesToRadians(m_camera.getSpeakerXOffsetDegrees()))) : Optional.empty();
  // }

  public double getRotationFromGoalRadians(Pose2d pose) {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get().toString().equals("Red"))
        return Math.atan(
          (pose.getY() - FieldConstants.kRedSpeakerAprilTagLocation.getY()) /
          Math.abs(pose.getX() - FieldConstants.kRedSpeakerAprilTagLocation.getX()) - Units.degreesToRadians(10)
        );
    else
      return Math.PI - Math.atan(
          (pose.getY() - FieldConstants.kBlueSpeakerAprilTagLocation.getY()) /
          Math.abs(pose.getX() - FieldConstants.kBlueSpeakerAprilTagLocation.getX()) - Units.degreesToRadians(10)
        );
    }
    return Math.atan(
          (pose.getY() - FieldConstants.kRedSpeakerAprilTagLocation.getY())  /
          Math.abs(pose.getX() - FieldConstants.kRedSpeakerAprilTagLocation.getX()) - Units.degreesToRadians(10)
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
   * @return 
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
    double xSpeedDelivered = xSpeedCommanded * maxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * maxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * maxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      ChassisSpeeds.discretize(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered),
          PeriodicConstants.kPeriodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond,
          speeds.omegaRadiansPerSecond,
          false,
          false);
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
        desiredStates, maxSpeedMetersPerSecond);
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

  public void setHeading(double angle) {
    m_gyro.setGyroAngleZ(angle);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees() % 360;
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
