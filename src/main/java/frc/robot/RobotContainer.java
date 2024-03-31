
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PhotonConstants;
import frc.robot.commands.IntakeCommand.IntakeState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.superstructure.StateMachine;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.StateMachine.ShooterState;
import frc.robot.subsystems.superstructure.StateMachine.TriggerState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutosCommands;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveToAmp;
// import frc.robot.commands.RotateToGoal;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotateToGoal;
//import frc.robot.commands.RotateToGoalProfiled;

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
  private final LED m_blinkin = new LED();
  private final Vision m_leftCamera = new Vision(PhotonConstants.kLeftCameraName, PhotonConstants.kLeftCameraLocation, m_blinkin);
  private final Vision m_rightCamera = new Vision(PhotonConstants.kRightCameraName, PhotonConstants.kRightCameraLocation);
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_leftCamera);
  // private final Shooter m_shooter = new Shooter(m_robotDrive);
  // private final Intake m_intake = new Intake();
  private final Amp m_amp = new Amp();
  // private final AutosCommands m_autosCommands = new AutosCommands(m_robotDrive, m_limelight, m_shooter, m_intake);
  private double kPThetaController = .3;//.7
  private SendableChooser<Command> autoChooser;
  private final Climber m_climber = new Climber();

  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  CommandXboxController m_configureController = new CommandXboxController(OIConstants.kConfigureControllerPort);

  private final Superstructure m_superstructure = new Superstructure(m_robotDrive, m_leftCamera, m_climber, m_amp, m_blinkin, m_driverController, m_operatorController);
  private final StateMachine m_stateMachine = new StateMachine(m_superstructure);

  ProfiledPIDController thetaController = new ProfiledPIDController(kPThetaController, 0, 0, new Constraints(10, 20));
  SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(2, 1);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    NamedCommands.registerCommand("intakeBack", m_stateMachine.intakeSelectCommand(StateMachine.IntakeState.INTAKE_BACK));
    NamedCommands.registerCommand("intakeFront", m_stateMachine.intakeSelectCommand(StateMachine.IntakeState.INTAKE_FRONT));
    NamedCommands.registerCommand("shoot", new SequentialCommandGroup(m_superstructure.shoot(), new WaitCommand(0.1), m_superstructure.stopShooter()));
    NamedCommands.registerCommand("stopShooter", m_superstructure.stopShooter());
    NamedCommands.registerCommand("stopIntake", m_stateMachine.intakeSelectCommand(StateMachine.IntakeState.IDLE));
    NamedCommands.registerCommand("setupSubwoofer", m_stateMachine.shooterSelectCommand(ShooterState.SUBWOOFER));
    NamedCommands.registerCommand("setupDynamic", m_stateMachine.enableDynamicShooter());
    // NamedCommands.registerCommand("setupAllianceZone", m_shooter.readyAllianceZone());
    // NamedCommands.registerCommand("setupSlow", new InstantCommand(() -> m_shooter.setFlywheelVelocity(1000)));
    // NamedCommands.registerCommand("setupFast", new InstantCommand(() -> m_shooter.setFlywheelVelocity(8000)));
    
    // NamedCommands.registerCommand("setDisruptorGyro", new InstantCommand(() -> m_robotDrive.setHeading(143.43)));
    // NamedCommands.registerCommand("setupClose", new ParallelCommandGroup(
    //                                                                       new InstantCommand(() -> m_shooter.setPivotAngle(45)),//tbd
    //                                                                       new InstantCommand(() -> m_shooter.setFlywheelVelocity(8000)))); //tbd
    NamedCommands.registerCommand("alignToGoal", new InstantCommand());
    // NamedCommands.registerCommand("pivot to 50", m_shooter.setPivotAngleCommand(30));
    NamedCommands.registerCommand("stowShooter", m_stateMachine.shooterSelectCommand(ShooterState.STOW)); //tbd
    NamedCommands.registerCommand("enableStoppedState", m_robotDrive.enableStopped());
    NamedCommands.registerCommand("disableStoppedState", m_robotDrive.disableStopped());

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

    m_driverController.rightTrigger(OIConstants.kTriggerThreshold)
      .onTrue(m_stateMachine.intakeSelectCommand(StateMachine.IntakeState.INTAKE_BACK))
      .onFalse(m_stateMachine.intakeSelectCommand(StateMachine.IntakeState.IDLE));
    m_driverController.leftTrigger(OIConstants.kTriggerThreshold)
      .onTrue(m_stateMachine.intakeSelectCommand(StateMachine.IntakeState.INTAKE_FRONT))
      .onFalse(m_stateMachine.intakeSelectCommand(StateMachine.IntakeState.IDLE));
    m_driverController.a()                                        
      .whileTrue(m_superstructure.shoot())
      .onFalse(m_superstructure.stopShooter());
    m_driverController.rightBumper().onTrue(m_robotDrive.setRotatingToGoalCommand());
    m_driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    m_driverController.b().whileTrue(new DriveToAmp(m_robotDrive, m_rightCamera));
    m_driverController.y()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    m_driverController.x().onTrue(new DriveStraight(m_robotDrive));

    m_operatorController.rightBumper()
      .onTrue(m_stateMachine.intakeSelectCommand(StateMachine.IntakeState.EXTAKE_BACK))
      .onFalse(m_stateMachine.intakeSelectCommand(StateMachine.IntakeState.IDLE));
    m_operatorController.leftBumper()
      .onTrue(m_stateMachine.intakeSelectCommand(StateMachine.IntakeState.EXTAKE_FRONT))
      .onFalse(m_stateMachine.intakeSelectCommand(StateMachine.IntakeState.IDLE));
    m_operatorController.y().onTrue(m_stateMachine.shooterSelectCommand(ShooterState.SOURCETOMID));
    m_operatorController.a().onTrue(m_stateMachine.shooterSelectCommand(ShooterState.STOW));
    m_operatorController.x().onTrue(m_stateMachine.shooterSelectCommand(ShooterState.UNDERSTAGE));
    m_operatorController.b().onTrue(m_stateMachine.shooterSelectCommand(ShooterState.DYNAMIC));
    m_operatorController.rightTrigger(OIConstants.kTriggerThreshold)
      .onTrue(m_stateMachine.shooterSelectCommand(ShooterState.SUBWOOFER));
    m_operatorController.leftTrigger(OIConstants.kTriggerThreshold)
      .onTrue(m_stateMachine.shooterSelectCommand(ShooterState.AMP));
    m_operatorController.povLeft().onTrue(m_stateMachine.shooterSelectCommand(ShooterState.PASSTOAMP));
    m_operatorController.povUp().onTrue(m_stateMachine.extendClimbers());
    m_operatorController.povDown().onTrue(m_stateMachine.retractClimbers());
    m_operatorController.povRight().onTrue(m_stateMachine.zeroClimbers());

    m_configureController.leftBumper().whileTrue(new ParallelCommandGroup(m_climber.extendLeftClimberToReset(), new InstantCommand(() -> m_amp.setPivot(0.1)))).whileFalse(m_climber.stopLeftClimber());
    m_configureController.rightBumper().whileTrue(new ParallelCommandGroup(m_climber.extendRightClimberToReset(), new InstantCommand(() -> m_amp.setPivot(0.1)))).whileFalse(m_climber.stopRightClimber());
    m_configureController.leftTrigger(OIConstants.kTriggerThreshold).whileTrue(new ParallelCommandGroup(m_climber.retractLeftClimberToReset(), new InstantCommand(() -> m_amp.setPivot(0.1)))).whileFalse(m_climber.stopLeftClimber());
    m_configureController.rightTrigger(OIConstants.kTriggerThreshold).whileTrue(new ParallelCommandGroup(m_climber.retractRightClimberToReset(), new InstantCommand(() -> m_amp.setPivot(0.1)))).whileFalse(m_climber.stopRightClimber());
    m_configureController.x().onTrue(m_climber.setLeftClimberZero());
    m_configureController.b().onTrue(m_climber.setRightClimberZero());
  }

  public void setTeleopDefaultStates() {
    m_stateMachine.shooterSelectCommand(ShooterState.STOW).schedule();
    m_stateMachine.setCurrentIntakeState(StateMachine.IntakeState.IDLE).schedule();
  }

  public void setAutonomousDefaultStates() {
    // new InstantCommand(() -> m_robotDrive.setHeading(180.0)).schedule();
    m_amp.stow().schedule();
  }

  public void setRobotDefaultStates() {
    m_blinkin.setFire();
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
