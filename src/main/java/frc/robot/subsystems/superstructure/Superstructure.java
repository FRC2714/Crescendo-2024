// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.RotateToGoal;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  Shooter m_shooter;
  Intake m_intake;
  DriveSubsystem m_drivetrain;
  Vision m_vision;
  Climber m_climber;
  Amp m_amp;
  LED m_blinkin;
  CommandXboxController m_driverController;
  CommandXboxController m_operatorController;

  double elapsedRumbleTime = 0;

  public Superstructure(DriveSubsystem m_drivetrain, Vision m_vision, Climber m_climber, Amp m_amp, LED m_blinkin, CommandXboxController m_driverController, CommandXboxController m_operatorController) {
    this.m_vision = m_vision;
    this.m_drivetrain = m_drivetrain;
    this.m_shooter = new Shooter(m_vision);
    this.m_intake = new Intake();
    this.m_climber = m_climber;
    this.m_amp = m_amp;
    this.m_blinkin = m_blinkin;
    this.m_driverController = m_driverController;
    this.m_operatorController = m_operatorController;
  }

  public boolean getLoaded() {
    return m_intake.getLoaded();
  }
  
  public Command resetLoadedAndIntakeBack(){
    return new SequentialCommandGroup(
      m_intake.disableLoaded(),
      m_intake.stopFront(),
      new IntakeCommand(m_intake, IntakeCommand.IntakeState.BACK)
        .until(() -> m_intake.getLoaded())
    );
  }
  public Command intakeBack() {
    return new SequentialCommandGroup(
      m_intake.stopFront(),
      new IntakeCommand(m_intake, IntakeCommand.IntakeState.BACK)
        .until(() -> m_intake.getLoaded())
    );
  }  
  public Command intakeBackRaw() {
    return new SequentialCommandGroup(
      m_intake.stopFront(),
      new IntakeCommand(m_intake, IntakeCommand.IntakeState.BACK)
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

  public Command stowAmp() {
    return m_amp.stow();
  }

  public Command deployAmp() {
    return m_amp.deploy();
  }


  public Command shootAndSetLoadedFalse(){
    return new SequentialCommandGroup(m_intake.shoot(),
                                      m_intake.disableLoaded());
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

  public Command readyShort() {
    return m_shooter.readyShort();
  }

  public Command readyPassToAmp() {
    return m_shooter.readyPassToAmp();
  }

     public Command readySourceToMid() {
    return m_shooter.readySourceToMid();
  }

      public Command readyUnderstage() {
    return m_shooter.readyUnderstage();
  }

  public Command readyShooterToAmp() {
    return new ParallelCommandGroup(m_shooter.readyAmp());
  }

  public Command stowShooter() {
    return new ParallelCommandGroup(m_shooter.stow());
  }

  public Command cancelAllCommands() {
    CommandScheduler.getInstance().cancelAll();
    return new InstantCommand();
  }

  public Command extendClimbers() {
    return m_climber.extendClimbersCommand();
  }

  public Command retractClimbers() {
    return m_climber.retractClimbersCommand();
  }

  public Command zeroClimbers() {
    return m_climber.zeroClimbersCommand();
  }

  public Command configureExtendLeftClimber() {
    return m_climber.extendLeftClimberToReset();
  }

  public Command configureExtendRightClimber() {
    return m_climber.extendRightClimberToReset();
  }

  public Command configureRetractLeftClimber() {
    return new ParallelCommandGroup(m_climber.retractLeftClimberToReset());
  }

  public Command configureRetractRightClimber() {
    return new ParallelCommandGroup(m_climber.retractRightClimberToReset());
  }

  public Command setCenterHeading() {
    return new InstantCommand(() -> m_drivetrain.setHeading(180));
  }

  // public Command setAmpSideHeading() {
  //   if (DriverStation.getAlliance().get().toString().equals("Blue"))
  //     return new InstantCommand(() -> m_drivetrain.setHeading(240));
  //   else
  //     return new InstantCommand(() -> m_drivetrain.setHeading(120));
  // }

  // public Command setSourceSideHeading() {
  //   if (DriverStation.getAlliance().get().toString().equals("Blue"))
  //     return new InstantCommand(() -> m_drivetrain.setHeading(120));
  //   else
  //     return new InstantCommand(() -> m_drivetrain.setHeading(240));
  // }

  public boolean isReadyToShoot() {
    return m_intake.getLoaded()
      && m_shooter.flywheelAtSetpoint()
      && Math.abs(m_vision.getSpeakerXOffsetDegrees()) < 10
      && m_vision.hasSpeakerTarget();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putString("Alliance", DriverStation.getAlliance().get().toString());
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

    if (isReadyToShoot()) {
      m_blinkin.setBlue();
    }
    else if (m_intake.getLoaded()) {
      m_blinkin.setGreen();
    }
    else {
      m_blinkin.setRed();
    }
  }
}
