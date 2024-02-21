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

public class AutosCommands {
  /** Creates a new AutosCommand. */
  private DriveSubsystem m_drivetrain = new DriveSubsystem();
  private Limelight m_limelight = new Limelight();
  private Shooter m_shooter = new Shooter(m_limelight);
  private Intake m_intake = new Intake();
  public Command setupShot(double shootingDistance) {
    return new InstantCommand(() -> {
    m_shooter.setPivotAngle(shootingDistance);
    m_shooter.setFlywheelVelocity(2500);});
  }
  public Command shoot() {
    return new InstantCommand(() -> {
    m_intake.setFeederVoltage(3);});

  }
}