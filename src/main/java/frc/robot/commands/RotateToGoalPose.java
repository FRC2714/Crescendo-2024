// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.ThetaPIDConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RotateToGoalPose extends Command {
  
  private DriveSubsystem m_drivetrain;

  private PIDController thetaController;
  private double rotationToGoal;

  /** Creates a new RotateToGoal. */
  public RotateToGoalPose(DriveSubsystem m_drivetrain) {
    this.m_drivetrain = m_drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);

    thetaController = new PIDController(ThetaPIDConstants.kP, ThetaPIDConstants.kI, ThetaPIDConstants.kD);
    thetaController.setTolerance(Units.degreesToRadians(0),0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationToGoal = m_drivetrain.getRotationFromGoalRadians(m_drivetrain.getPose());
    thetaController.setSetpoint(-rotationToGoal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(
        0, 
        0, 
        thetaController.calculate(m_drivetrain.getPose().getRotation().getRadians()), 
        true,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return thetaController.atSetpoint();
  }
}