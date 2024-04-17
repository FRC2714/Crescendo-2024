// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PhotonConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveToAmp extends Command {
  private DriveSubsystem m_drivetrain;
  private Vision m_camera;

  private PIDController thetaController;
  private PIDController xController, yController;

  private double kPThetaController = 0.5;
  private double kPXController = 0.8;
  private double kPYController = 0.5;


  /** Creates a new DriveToAmp. */
  public DriveToAmp(DriveSubsystem m_drivetrain, Vision m_camera) {
    this.m_drivetrain = m_drivetrain;
    this.m_camera = m_camera;

    // Use addRequirements() here to declare subsystem dependencies.
    thetaController = new PIDController(kPThetaController, 0, 0);
    xController = new PIDController(kPXController,0,0);
    yController = new PIDController(kPYController,0,0);
    
    addRequirements(m_drivetrain, m_camera);

    thetaController.setSetpoint(0);
    thetaController.setTolerance(Units.degreesToRadians(0),0);
    xController.setSetpoint(0);
    xController.setTolerance(Units.degreesToRadians(0),0);

    yController.setSetpoint(0.14);
    yController.setTolerance(Units.degreesToRadians(0),0);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_camera.ampVisible()) {
      m_drivetrain.drive(
      -xController.calculate(m_camera.getXDistanceMeters()), 
      -yController.calculate(m_camera.getYDistanceMeters()),
      thetaController.calculate(Units.degreesToRadians(m_camera.getAmpXOffsetDegrees())),
      false,
      false);
    }
    else {
      m_drivetrain.drive(0, 0, 0, true, false);
    }
  }  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return thetaController.atSetpoint() && xController.atSetpoint() && yController.atSetpoint();
  }
}