// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DriveConstants.ThetaPIDConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.utils.FieldRelativeAcceleration;
import frc.utils.FieldRelativeVelocity;

public class MoveAndShoot extends Command {
  /** Creates a new MoveAndShoot. */
  private DriveSubsystem m_drivetrain;
  private Limelight m_limelight;
  private Shooter m_shooter;

  private CommandXboxController m_controller;

  private PIDController thetaController;

  public MoveAndShoot(DriveSubsystem m_drivetrain, Limelight m_limelight, Shooter m_shooter, CommandXboxController m_controller) {
    this.m_drivetrain = m_drivetrain;
    this.m_limelight = m_limelight;
    this.m_shooter = m_shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain, m_limelight, m_shooter);

    thetaController = new PIDController(ThetaPIDConstants.kP, ThetaPIDConstants.kP, ThetaPIDConstants.kP);

    thetaController.setSetpoint(0);
    thetaController.setTolerance(Units.degreesToRadians(0),0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setSpeakerPipeline();
    m_limelight.setLED(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_limelight.isTargetVisible()) {

      FieldRelativeVelocity fieldRelVel = m_drivetrain.getFieldRelativeVelocity();
      FieldRelativeAcceleration fieldRelAccel = m_drivetrain.getFieldRelativeAcceleration();

      double getDistanceToGoalMeters = m_limelight.getDistanceToGoalMeters();

      double adjustedXOffsetRadians = m_limelight.getXOffsetRadians() -
        m_shooter.getInterpolatedShootTime(getDistanceToGoalMeters) * 
        (fieldRelVel.vx + (fieldRelAccel.ax * ShooterConstants.kAccelerationCompensationFactor));
      
      double adjustedYOffsetRadians = m_limelight.getYOffsetRadians() -
        m_shooter.getInterpolatedShootTime(getDistanceToGoalMeters) * 
        (fieldRelVel.vy + (fieldRelAccel.ay * ShooterConstants.kAccelerationCompensationFactor));

      double adjustedDistanceToGoal = m_limelight.getDistanceToGoalMeters(adjustedYOffsetRadians);

      m_drivetrain.drive(
        -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband),
        thetaController.calculate(adjustedXOffsetRadians), 
        true,
        true);
      
      m_shooter.setMoveAndShoot(adjustedDistanceToGoal);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limelight.setLED(false);
    m_drivetrain.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
