// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IntakeCommand.IntakeState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.superstructure.StateMachine;
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
  // private final Limelight m_limelight = new Limelight();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // private final Shooter m_shooter = new Shooter(m_robotDrive);
  // private final Intake m_intake = new Intake();
  // private final Amp m_amp = new Amp();
  // private final AutosCommands m_autosCommands = new AutosCommands(m_robotDrive, m_limelight, m_shooter, m_intake);
  private double kPThetaController = .7;
  private SendableChooser<Command> autoChooser;
  // private final Climber m_climber = new Climber();

  private final StateMachine m_stateMachine = new StateMachine(m_robotDrive);

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  CommandXboxController m_configureController = new CommandXboxController(OIConstants.kConfigureControllerPort);

  ProfiledPIDController thetaController = new ProfiledPIDController(kPThetaController, 0, 0, new Constraints(10, 20));
  SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(2, 1);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    // NamedCommands.registerCommand("intakeBackBeam", new IntakeCommand(m_intake, IntakeState.BACK).until(() -> m_intake.getLoaded()));
    // NamedCommands.registerCommand("intakeFrontBeam", new IntakeCommand(m_intake, IntakeState.FRONT).until(() -> m_intake.getLoaded()));
    // NamedCommands.registerCommand("shoot", m_intake.shoot().withTimeout(3));
    // NamedCommands.registerCommand("intakeBack", m_intake.intakeBack());
    // NamedCommands.registerCommand("intakeFront", m_intake.intakeFront());
    // NamedCommands.registerCommand("stopIntakeBack", m_intake.stopBack());
    // NamedCommands.registerCommand("stopIntakeFront", m_intake.stopFront());
    // NamedCommands.registerCommand("setupShort", m_shooter.setupShot(37));
    // NamedCommands.registerCommand("setupDynamic", new InstantCommand(() -> m_shooter.toggleDynamic()));
    // NamedCommands.registerCommand("setupSubwoofer", m_shooter.readySubwoofer());
    // NamedCommands.registerCommand("setupAllianceZone", m_shooter.readyAllianceZone());
    // NamedCommands.registerCommand("setupSlow", new InstantCommand(() -> m_shooter.setFlywheelVelocity(1000)));
    // NamedCommands.registerCommand("setupFast", new InstantCommand(() -> m_shooter.setFlywheelVelocity(8000)));
    
    // NamedCommands.registerCommand("setDisruptorGyro", new InstantCommand(() -> m_robotDrive.setHeading(143.43)));
    // NamedCommands.registerCommand("setupClose", new ParallelCommandGroup(
    //                                                                       new InstantCommand(() -> m_shooter.setPivotAngle(45)),//tbd
    //                                                                       new InstantCommand(() -> m_shooter.setFlywheelVelocity(8000)))); //tbd
    // NamedCommands.registerCommand("alignToGoal", m_robotDrive.toggleRotatingToGoalCommand().withTimeout(3));
    // NamedCommands.registerCommand("pivot to 50", m_shooter.setPivotAngleCommand(30));
    // NamedCommands.registerCommand("stowShooter", m_shooter.stow()); //tbd

    autoChooser = AutoBuilder.buildAutoChooser("3 Note Auto Top");
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure default commands
    // m_robotDrive.setDefaultCommand(
    //     // The left stick controls translation of the robot.
    //     // Turning is controlled by the X axis of the right stick.
    //     new RunCommand(
    //         () -> m_robotDrive.drive(
    //             -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
    //             m_robotDrive.getRotatingToGoal(-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband))
    //                     ? m_robotDrive.getDriveRotationToGoal()
    //                     : -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
    //             true, false),
    //         m_robotDrive));
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

    m_driverController.rightTrigger(OIConstants.kTriggerThreshold)
      .whileTrue(m_stateMachine.intakeSelectCommand(StateMachine.IntakeState.BACK))
      .onFalse(m_stateMachine.intakeSelectCommand(StateMachine.IntakeState.IDLE));
    m_driverController.leftTrigger(OIConstants.kTriggerThreshold)
      .whileTrue(m_stateMachine.intakeSelectCommand(StateMachine.IntakeState.FRONT))
      .onFalse(m_stateMachine.intakeSelectCommand(StateMachine.IntakeState.IDLE));

    // m_driverController.rightTrigger(OIConstants.kTriggerThreshold).whileTrue(new IntakeCommand(m_intake, IntakeState.BACK).until(() -> m_intake.getLoaded()));
    // m_driverController.leftTrigger(OIConstants.kTriggerThreshold).whileTrue(new IntakeCommand(m_intake, IntakeState.FRONT).until(() -> m_intake.getLoaded()));
    // m_driverController.rightBumper().onTrue(m_robotDrive.setRotatingToGoalCommand());
    // m_driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    // m_driverController.a().whileTrue(m_intake.shoot()).onFalse(m_intake.stopShooter());
    // m_driverController.b().whileTrue(m_shooter.setFlywheelVelocityCommand(8000)).onFalse(m_shooter.setFlywheelVelocityCommand(0));
    // m_driverController.leftBumper().whileTrue(new ParallelCommandGroup(new SeekNote(m_robotDrive, m_limelight),
    //                                           new IntakeCommand(m_intake, IntakeState.BACK).until(() -> m_intake.getLoaded())));
    m_driverController.y()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    // m_operatorController.rightTrigger(OIConstants.kTriggerThreshold).whileTrue(new IntakeCommand(m_intake, IntakeState.BACK).until(() -> m_intake.getLoaded()));
    // m_operatorController.leftTrigger(OIConstants.kTriggerThreshold).whileTrue(new IntakeCommand(m_intake, IntakeState.FRONT).until(() -> m_intake.getLoaded()));
    // m_operatorController.rightBumper().whileTrue(m_intake.outtakeBack()).whileFalse(m_intake.stopBack());
    // m_operatorController.leftBumper().whileTrue(m_intake.outtakeFront()).whileFalse(m_intake.stopFront());
    // m_operatorController.b().onTrue(new ParallelCommandGroup(m_shooter.stow(), m_amp.stow()));
    // m_operatorController.x().onTrue(m_shooter.readySubwoofer());
    // m_operatorController.y().onTrue(m_shooter.readyPass());
    // m_operatorController.back().onTrue(new InstantCommand(() -> m_shooter.toggleDynamic()));
    // m_operatorController.a().onTrue(new ParallelCommandGroup(m_shooter.readyAmp(), m_amp.extend()));
    // m_operatorController.povUp().whileTrue(new ParallelCommandGroup(m_climber.extendClimbersCommand().until(() -> m_climber.rightClimberAtMax()), new InstantCommand(() -> m_amp.setPivot(0.1)))).whileFalse(m_climber.stopClimbersCommand());
    // m_operatorController.povDown().whileTrue(new ParallelCommandGroup(m_climber.retractClimbersCommand().until(() -> m_climber.rightClimberAtMin()), new InstantCommand(() -> m_amp.setPivot(0.1)))).whileFalse(m_climber.stopClimbersCommand());                      
    // m_operatorController.povRight().onTrue(m_shooter.incrementPivotAngle());
    // m_operatorController.povLeft().onTrue(m_shooter.decrementPivotAngle());


    // m_configureController.leftBumper().whileTrue(new ParallelCommandGroup(m_climber.extendLeftClimberToReset(), new InstantCommand(() -> m_amp.setPivot(0.1)))).whileFalse(m_climber.stopLeftClimber());
    // m_configureController.rightBumper().whileTrue(new ParallelCommandGroup(m_climber.extendRightClimberToReset(), new InstantCommand(() -> m_amp.setPivot(0.1)))).whileFalse(m_climber.stopRightClimber());
    // m_configureController.leftTrigger(0.1).whileTrue(new ParallelCommandGroup(m_climber.retractLeftClimberToReset(), new InstantCommand(() -> m_amp.setPivot(0.1)))).whileFalse(m_climber.stopLeftClimber());
    // m_configureController.rightTrigger(0.1).whileTrue(new ParallelCommandGroup(m_climber.retractRightClimberToReset(), new InstantCommand(() -> m_amp.setPivot(0.1)))).whileFalse(m_climber.stopRightClimber());
    // m_configureController.x().onTrue(m_climber.setLeftClimberZero());
    // m_configureController.b().onTrue(m_climber.setRightClimberZero());
  }

  // public void setTeleopDefaultStates() {
  //   new ParallelCommandGroup(m_shooter.setPivotAngleCommand(0),
  //     m_shooter.setFlywheelVelocityCommand(0)).schedule();
  // }

  // public void setAutonomousDefaultStates() {
  //   new InstantCommand(() -> m_robotDrive.setHeading(180.0)).schedule();
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
