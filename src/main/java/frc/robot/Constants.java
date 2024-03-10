// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ClimberConstants {
    public static final int kLeftClimberCanId = 17; // TBD
    public static final int kRightClimberCanId = 18; // TBD
    
    public static final int kLeftClimberSmartCurrentLimit = 80; // TBD
    public static final int kRightClimberSmartCurrentLimit = 80; // TBD

    public static final double kMaxExtension = 125; // TBD
    public static final double kMinExtension = -72; // TBD
    public static final double kClimberVoltage = 8;
  }


  public static final class IntakeConstants {
    public static final int kFrontRollerCanId = 9;
    public static final int kBackBottomRollerCanId = 10;
    public static final int kBackDirectionRollerCanId = 11;
    public static final int kConveyorCanId = 12;
    public static final int kFeederCanId = 16;

    public static final int kBreakBeamChannel = 0;

    public static final double kRollerNominalVoltage = 11;

    public static final int kFrontRollerSmartCurrentLimit = 80; // TBD
    public static final int kBackBottomRollerSmartCurrentLimit = 80; // TBD
    public static final int kBackDirectionRollerSmartCurrentLimit = 80; // TBD
    public static final int kConveyorSmartCurrentLimit = 80; // TBD
    public static final int kFeederSmartCurrentLimit = 80;

    public static final double kFrontRollerVoltage = 5; // TBD
    public static final double kBackBottomRollerVoltage = 3; // TBD
    public static final double kBackDirectionRollerVoltageFrontSide = 3; // TBD
    public static final double kFrontDirectionRollerVoltageFrontSide = 3; // TBD
    public static final double kBackBottomRollerVoltageFrontSide = 3;
    public static final double kBackDirectionRollerVoltageBackSide = 3; // TBD
    public static final double kBackBottomRollerVoltageBackSide = 3;
    public static final double kFrontDirectionRollerVoltageBackSide = 3; // TBD
    public static final double kConveyorVoltage = 5; // TBD
    public static final double kFeederVoltage = 2; // TBD
  }


  public static final class LimelightConstants {
    public static final Pose3d kBackLimelightPose = 
      new Pose3d(
        new Translation3d(-16.2816, 0, 10.3143), //inches
        new Rotation3d(0.0, Units.degreesToRadians(-21.0), 0.0));//degrees
    public static final String kLimelightName = "limelight-back";

    public static final double kNoteGoalHeight = 0; //inches, deg NEEDS TO BE UPDATED
  }

  public static final class LEDConstants {
    public static final int kArmBlinkinPort = 0;

    public static final double kFire = -0.57;
    public static final double kRainbow = -0.99;
    public static final double kSinelonLava = -0.73;

    public static final double kPurple = 0.91;
    public static final double kYellow = 0.69;
    public static final double kGreen = 0.77;
    public static final double kRed = 0.61;
    public static final double kGold = 0.67;
  }

  public static final class ShooterConstants {
    public static final int kTopFlywheelCanId = 14;
    public static final int kBottomFlywheelCanId = 15;
    public static final int kPivotCanId = 13;

    public static final double kPivotGearRatio = 25;
    public static final double kFlywheelGearRatio = 2;

    public static final double kPivotEncoderZeroOffset = 192 * kPivotGearRatio;
    public static final double kPivotEncoderKinematicOffset = 10 * kPivotGearRatio;

    public static final double kMinPivotAngle = 0;
    public static final double kMaxPivotAngle = 80;

    public static final double kPivotEncoderConversionFactor = 360 * kPivotGearRatio;

    public static final int kPivotSmartCurrentLimit = 40; // TBD
    public static final int kFollowingPivotSmartCurrentLimit = 40; // TBD
    public static final int kTopFlywheelSmartCurrentLimit = 40; // TBD
    public static final int kBottomFlywheelSmartCurrentLimit = 40; // TBD

    public static final double kAccelerationCompensationFactor = 0.1;

    public static final double kNominalVoltage = 11; // TBD

    public static final double kAmpAngle = 25;
    public static final double kAmpFlywheelVelocity = 2500;

    public static final double kSubwooferAngle = 40;
    public static final double kSubwooferFlywheelVelocity = 8000;

    public static final class PivotPIDConstants {
      public static final double kP = 0.45; // TBD
      public static final double kI = 0; // TBD
      public static final double kD = 0; // TBD
    }
    
    public static final class FlywheelPIDConstants {
      public static final double kP = 0.000007; // TBD
      public static final double kI = 0; // TBD
      public static final double kD = 0; // TBD

      public static final double kS = 0; // TBD
      public static final double kV = 0; // TBD
      public static final double kA = 0; // TBD

      public static final double kFF = 0.00009;
    }
  }

  public static final class PhotonConstants {
    public static final String kBackCameraName = "backCamera";
    public static final String kFrontCameraName = "frontCamera";
    public static final Transform3d kBackCameraLocation = new Transform3d(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(-11.000111), Units.inchesToMeters(12.17225)),
                                                                          new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(180)));
    public static final Transform3d kFrontCameraLocation = new Transform3d(new Translation3d(Units.inchesToMeters(-15), Units.inchesToMeters(12.249889), Units.inchesToMeters(13.018624)),
                                                                          new Rotation3d(0, Units.degreesToRadians(-20), 0));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class FieldConstants {
    public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final Pose2d kBlueSpeakerAprilTagLocation = new Pose2d(0.0381, 5.547, new Rotation2d(0));
    public static final Pose2d kRedSpeakerAprilTagLocation = new Pose2d(16.57, 5.547, new Rotation2d(0));
  }
  public static final class AmpConstants {
      public static final int kLeftAmpPivotChannel = 1;
      public static final int kRightAmpPivotChannel = 0;
    }

  public static final class DriveConstants {

    public static final Pose2d kInitialRedPose = new Pose2d(15.57, 5.547, new Rotation2d(0));
    public static final Pose2d kInitialBluePose = new Pose2d(1.0381, 5.547, new Rotation2d(180));
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 7.59864;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(28.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(16.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // CAN IDs
    public static final int kFrontLeftDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 2;


    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 6;

    public static final boolean kGyroReversed = false;

    // Swerve PID values
    public static final class ThetaPIDConstants {
      public static final double kP = 1;
      public static final double kI = 0;
      public static final double kD = 0;
    }

    public static final class RollPIDConstants {
      public static final double kP = 0; // TBD
      public static final double kI = 0; // TBD
      public static final double kD = 0; // TBD
    }
  }
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 16;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoVortexMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 19 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 19) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kConfigureControllerPort = 2;
    public static final double kDriveDeadband = 0.05;
    public static final double kTriggerThreshold = 0.1;
    public static final double kRumble = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
  
  public static final class NeoVortexMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }
}