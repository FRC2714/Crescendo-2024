// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IntakeCommand.IntakeState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutosCommands;
// import frc.robot.commands.RotateToGoal;
import frc.robot.commands.IntakeCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.SeekNote;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Limelight m_limelight = new Limelight();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Shooter m_shooter = new Shooter(m_robotDrive);
  private final Intake m_intake = new Intake();
  private final AutosCommands m_autosCommands = new AutosCommands(m_robotDrive, m_limelight, m_shooter, m_intake);
  private double kPThetaController = .7;
  private SendableChooser<Command> autoChooser;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  private final Amp m_amp = new Amp();



  ProfiledPIDController thetaController = new ProfiledPIDController(kPThetaController, 0, 0, new Constraints(10, 20));
  SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(2, 1);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    NamedCommands.registerCommand("intakeBack", m_intake.intakeBack());
    NamedCommands.registerCommand("intakeFront", m_intake.intakeFront());
    NamedCommands.registerCommand("stopIntakeBack", m_intake.stopBack());
    NamedCommands.registerCommand("stopIntakeFront", m_intake.stopFront());
   

    NamedCommands.registerCommand("setupShort", m_shooter.setupShot(37));
    NamedCommands.registerCommand("setupDynamic", new InstantCommand(() -> m_shooter.toggleDynamic()));
    NamedCommands.registerCommand("setupSlow", new InstantCommand(() -> m_shooter.setFlywheelVelocity(1000)));
    NamedCommands.registerCommand("setupClose", new ParallelCommandGroup(
                                                                          new InstantCommand(() -> m_shooter.setPivotAngle(45)),//tbd
                                                                          new InstantCommand(() -> m_shooter.setFlywheelVelocity(8000)))); //tbd
    NamedCommands.registerCommand("alignToGoal", m_robotDrive.toggleRotatingToGoalCommand());
    NamedCommands.registerCommand("shoot", m_autosCommands.shoot());



    NamedCommands.registerCommand("pivot to 50", m_shooter.setPivotAngleCommand(30));
    NamedCommands.registerCommand("stowShooter", m_shooter.stow()); //tbd



    
    autoChooser = AutoBuilder.buildAutoChooser("3 Note Auto Top");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                m_robotDrive.getRotatingToGoal(-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband))
                        ? m_robotDrive.getDriveRotationToGoal()
                        : -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, false),
            m_robotDrive));
    }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    m_driverController.rightTrigger(OIConstants.kTriggerThreshold).whileTrue(new IntakeCommand(m_intake, IntakeState.BACK).until(() -> m_intake.getLoaded()));
    m_driverController.leftTrigger(OIConstants.kTriggerThreshold).whileTrue(new IntakeCommand(m_intake, IntakeState.FRONT).until(() -> m_intake.getLoaded()));

    m_operatorController.rightTrigger(OIConstants.kTriggerThreshold).whileTrue(new IntakeCommand(m_intake, IntakeState.BACK).until(() -> m_intake.getLoaded()));
    m_operatorController.leftTrigger(OIConstants.kTriggerThreshold).whileTrue(new IntakeCommand(m_intake, IntakeState.FRONT).until(() -> m_intake.getLoaded()));

    m_operatorController.rightBumper().whileTrue(m_intake.outtakeBack()).whileFalse(m_intake.stopBack());
    m_operatorController.leftBumper().whileTrue(m_intake.outtakeFront()).whileFalse(m_intake.stopFront());

    m_driverController.rightBumper().onTrue(m_robotDrive.setRotatingToGoalCommand());
    
    m_driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    m_driverController.a().whileTrue(m_intake.shoot()).onFalse(m_intake.stopShooter());
    m_driverController.b().whileTrue(m_shooter.setFlywheelVelocityCommand(8000)).onFalse(m_shooter.setFlywheelVelocityCommand(0));

    m_operatorController.x().onTrue(m_amp.extend()).onFalse(m_amp.stow());
    m_driverController.y()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    m_operatorController.povDown().onTrue(m_shooter.stow());
    m_operatorController.povLeft().onTrue(new InstantCommand(() -> m_shooter.toggleDynamic()));
    m_operatorController.povUp().onTrue(m_shooter.readyAmp());
    m_driverController.leftBumper().whileTrue(new ParallelCommandGroup(new SeekNote(m_robotDrive, m_limelight),
                                              new IntakeCommand(m_intake, IntakeState.BACK).until(() -> m_intake.getLoaded())));
  }

  public void setTeleopDefaultStates() {
    m_shooter.setPivotAngleCommand(0);
    m_shooter.setFlywheelVelocityCommand(0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
