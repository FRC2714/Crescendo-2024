// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DriveConstants.ThetaPIDConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.FieldRelativeAcceleration;
import frc.robot.utils.FieldRelativeVelocity;

public class MoveAndShootPose extends Command {
  /** Creates a new MoveAndShoot. */
  private DriveSubsystem m_drivetrain;
  private Shooter m_shooter;

  private CommandXboxController m_controller;

  private PIDController thetaController;

  private Pose2d robotPose;
  private double xDistanceToGoal, yDistanceToGoal, rotationToGoal, distanceToGoal;

  private Translation3d speakerGoal;

  public MoveAndShootPose(DriveSubsystem m_drivetrain, Shooter m_shooter, CommandXboxController m_controller) {
    this.m_drivetrain = m_drivetrain;
    this.m_shooter = m_shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain, m_shooter);

    speakerGoal = String.valueOf(SmartDashboard.getData("Alliance Selection")).equals("Blue")
                  ? FieldConstants.kBlueSpeakerAprilTagLocation
                  : FieldConstants.kRedSpeakerAprilTagLocation;

    thetaController = new PIDController(ThetaPIDConstants.kP, ThetaPIDConstants.kP, ThetaPIDConstants.kP);

    thetaController.setSetpoint(0);
    thetaController.setTolerance(Units.degreesToRadians(0),0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    FieldRelativeVelocity fieldRelVel = m_drivetrain.getFieldRelativeVelocity();
    FieldRelativeAcceleration fieldRelAccel = m_drivetrain.getFieldRelativeAcceleration();

    robotPose = m_drivetrain.getPose();

    xDistanceToGoal = robotPose.getX() - speakerGoal.getX();
    yDistanceToGoal = robotPose.getY() - speakerGoal.getY();

    distanceToGoal = Math.sqrt(Math.pow(xDistanceToGoal, 2) + Math.pow(yDistanceToGoal, 2));

    rotationToGoal = Math.atan(xDistanceToGoal / yDistanceToGoal);

    rotationToGoal = robotPose.getRotation().getRadians() > Math.PI ? -(2 * Math.PI - robotPose.getRotation().getRadians()) : robotPose.getRotation().getRadians() - rotationToGoal;

    double adjustedRotationToGoal = rotationToGoal -
        m_shooter.getInterpolatedShootTime(distanceToGoal) * 
        (fieldRelVel.vx + (fieldRelAccel.ax * ShooterConstants.kAccelerationCompensationFactor));
      
    double adjustedDistanceToGoal = yDistanceToGoal - 
      m_shooter.getInterpolatedShootTime(distanceToGoal) * 
      (fieldRelVel.vy + (fieldRelAccel.ay * ShooterConstants.kAccelerationCompensationFactor));

    m_drivetrain.drive(
        -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband),
        thetaController.calculate(adjustedRotationToGoal), 
        true,
        true);
      
    m_shooter.setMoveAndShoot(adjustedDistanceToGoal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
