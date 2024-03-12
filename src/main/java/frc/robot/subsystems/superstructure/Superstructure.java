// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  Shooter m_shooter;
  Intake m_intake;
  DriveSubsystem m_drivetrain;
  Vision m_vision;
  CommandXboxController m_driverController;
  CommandXboxController m_operatorController;

  double elapsedRumbleTime = 0;

  public Superstructure(DriveSubsystem m_drivetrain, Vision m_vision, CommandXboxController m_driverController, CommandXboxController m_operatorController) {
    this.m_vision = m_vision;
    this.m_drivetrain = m_drivetrain;
    this.m_shooter = new Shooter(m_vision);
    this.m_intake = new Intake();
    this.m_driverController = m_driverController;
    this.m_operatorController = m_operatorController;
  }

  public boolean getLoaded() {
    return m_intake.getLoaded();
  }

  public Command intakeBack() {
    return new SequentialCommandGroup(
      m_intake.stopFront(),
      new IntakeCommand(m_intake, IntakeCommand.IntakeState.BACK)
        .until(() -> m_intake.getLoaded())
    );
  }

  public Command intakeFront() {
    return new SequentialCommandGroup(
      m_intake.stopBack(),
      new IntakeCommand(m_intake, IntakeCommand.IntakeState.FRONT)
        .until(() -> m_intake.getLoaded())
    );
  }

  public Command extakeBack() {
    return new SequentialCommandGroup(
      m_intake.stopFront(),
      m_intake.outtakeBack()
    );
  }

  public Command extakeFront() {
    return new SequentialCommandGroup(
      m_intake.stopBack(),
      m_intake.outtakeFront()
    );
  }

  public Command shoot() {
    return m_intake.shoot();
  }

  public Command stopShooter() {
    return new SequentialCommandGroup(
      m_intake.stopShooter(),
      m_intake.disableLoaded()
    );
  }

  public Command idleIntake() {
    return new ParallelCommandGroup(
      m_intake.stopBack(),
      m_intake.stopFront()
    );
  }

  public Command idleExtake() {
    return new ParallelCommandGroup(
      m_intake.stopBack(),
      m_intake.stopFront(),
      m_intake.disableLoaded()
    );
  }

  public Command setReadyToShoot() {
    return new ParallelCommandGroup(
      m_intake.stopBack(),
      m_intake.stopFront()
    );
  }

  public Command enableDynamicShooter() {
    return m_shooter.enableDynamic();
  }

  public Command disableDynamicShooter() {
    return m_shooter.disableDynamic();
  }

  public Command readyShooterToSubwoofer() {
    return m_shooter.readySubwoofer();
  }

  public Command readyShooterToAmp() {
    return m_shooter.readyAmp();
  }

  public Command stowShooter() {
    return m_shooter.stow();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getLoaded() && elapsedRumbleTime < OIConstants.kRumbleTimeMS) {
      elapsedRumbleTime += 20;
      m_driverController.getHID().setRumble(RumbleType.kBothRumble, OIConstants.kRumblePower);
      m_operatorController.getHID().setRumble(RumbleType.kBothRumble, OIConstants.kRumblePower);
    }
    else {
      if (!getLoaded()) elapsedRumbleTime = 0;
      m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
      m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
  }
}
