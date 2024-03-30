// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveStraight extends Command {
  /** Creates a new DriveStraight. */

  DriveSubsystem m_drivetrain;
  double time;
  public DriveStraight(DriveSubsystem m_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = m_drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(0.1, 0, 0, true, false);
    time += 0.02;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time >= 5;
  }
}
