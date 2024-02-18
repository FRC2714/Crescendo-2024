// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.commands.RotateToGoal;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
  private final Shooter m_shooter = new Shooter(m_limelight);
  private final Intake m_intake = new Intake();
  private double kPThetaController = .7;
  private final SendableChooser<Command> autoChooser;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);


  ProfiledPIDController thetaController = new ProfiledPIDController(kPThetaController, 0, 0, new Constraints(10, 20));
  SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(2, 1);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("intakeFront", m_intake.intakeFront());
    NamedCommands.registerCommand("intakeBack", m_intake.intakeBack());

    NamedCommands.registerCommand("shoot", m_shooter.setFlywheelVelocityCommand(3000));



    NamedCommands.registerCommand("intakeFront", m_intake.intakeFront());
    NamedCommands.registerCommand("intakeBack", m_intake.intakeBack());
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
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
    m_driverController.rightBumper().whileTrue(m_intake.intakeBack()).whileFalse(m_intake.stopBack());
    m_driverController.leftBumper().whileTrue(m_intake.intakeFront()).whileFalse(m_intake.stopFront());

    m_driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    m_driverController.povUp().onTrue(m_shooter.setFlywheelVelocityCommand(3000));
    m_driverController.povDown().onTrue(m_shooter.setFlywheelVelocityCommand(0));
    m_driverController.b().whileTrue(m_intake.outtakeBack()).onFalse(m_intake.stopBack());
    m_driverController.a().whileTrue(m_intake.outtakeFront()).onFalse(m_intake.stopFront());
    m_driverController.y()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
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
