// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.ThetaPIDConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RotateToGoal extends Command {
  
  private DriveSubsystem m_drivetrain;
  private Limelight m_limelight;

  private PIDController thetaController;

  /** Creates a new RotateToGoal. */
  public RotateToGoal(DriveSubsystem m_drivetrain, Limelight m_limelight) {
    this.m_drivetrain = m_drivetrain;
    this.m_limelight = m_limelight;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain, m_limelight);

    thetaController = new PIDController(ThetaPIDConstants.kP, ThetaPIDConstants.kP, ThetaPIDConstants.kP);

    thetaController.setSetpoint(0);
    thetaController.setTolerance(Units.degreesToRadians(0),0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setStagePipeline();
    m_limelight.setLED(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_limelight.isTargetVisible()) {
      m_drivetrain.drive(
        0, 
        0, 
        thetaController.calculate(m_limelight.getXOffsetRadians()), 
        true,
        true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limelight.setLED(false);
    m_drivetrain.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return thetaController.atSetpoint();
  }
}