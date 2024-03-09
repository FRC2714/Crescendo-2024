// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  Shooter m_shooter;
  Intake m_intake;
  DriveSubsystem m_drivetrain;

  public Superstructure(DriveSubsystem m_drivetrain) {
    this.m_drivetrain = m_drivetrain;
    this.m_shooter = new Shooter(m_drivetrain);
    this.m_intake = new Intake();
  }

  public Command intakeBack() {
    return new SequentialCommandGroup(
      m_intake.stopFront(),
      m_intake.intakeBack()
    );
  }

  public Command intakeFront() {
    return new SequentialCommandGroup(
      m_intake.stopBack(),
      m_intake.intakeFront()
    );
  }

  public Command shoot() {
    return new SequentialCommandGroup(
      m_intake.disableLoaded(),
      m_intake.shoot()
    );
  }

  public Command idle() {
    return new ParallelCommandGroup(
      m_intake.stopBack(),
      m_intake.stopFront(),
      m_shooter.stow()
    );
  }

  public Command setReadyToShoot() {
    return new ParallelCommandGroup(
      m_intake.stopBack(),
      m_intake.stopFront()
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
