// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.TimeUnit;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutosCommands extends Command{
  /** Creates a new AutosCommand. */
  private DriveSubsystem m_drivetrain;
  private Limelight m_limelight;
  private Shooter m_shooter;
  private Intake m_intake;
  public AutosCommands(DriveSubsystem m_drivetrain, Limelight m_limelight, Shooter m_shooter, Intake m_intake){
    this.m_drivetrain = m_drivetrain;
    this.m_limelight = m_limelight;
    this.m_shooter = m_shooter;
    this.m_intake = m_intake;
    addRequirements(m_drivetrain, m_limelight, m_shooter, m_intake);
  }
  public Command shoot() {
    return new InstantCommand(() -> {
      m_shooter.setPivotAngle(35);
    m_shooter.setFlywheelVelocity(1000);
    try {
      TimeUnit.SECONDS.sleep(1);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    m_intake.setFeederVoltage(3);});

  }
}