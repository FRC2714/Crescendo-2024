// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.RollPIDConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoBalance extends Command {
  /** Creates a new AutoBalance. */

  private Climber m_climber;
  private DriveSubsystem m_drive;

  private PIDController rollController;

  public AutoBalance(Climber m_climber, DriveSubsystem m_drive) {
  
    this.m_climber = m_climber;
    this.m_drive = m_drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber, m_drive);

    rollController = new PIDController(RollPIDConstants.kP, RollPIDConstants.kI, RollPIDConstants.kD);

    rollController.setSetpoint(0);
    rollController.setTolerance(Units.degreesToRadians(0), 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.setLeftClimber(m_drive.getRoll());
    m_climber.setRightClimber(m_drive.getRoll());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
