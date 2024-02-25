// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SeekNote extends Command {
  private DriveSubsystem m_drivetrain;
  private Limelight m_limelight;

  private ProfiledPIDController thetaController;
  private ProfiledPIDController xController;

  private double kPThetaController = 1;
  private double kPXController = 0.85;


  /** Creates a new RotateToGoal. */
  public SeekNote(DriveSubsystem m_drivetrain, Limelight m_limelight) {
    this.m_drivetrain = m_drivetrain;
    this.m_limelight = m_limelight;

    // Use addRequirements() here to declare subsystem dependencies.
    thetaController = new ProfiledPIDController(kPThetaController, 0, 0, new Constraints(4, 4));
    xController = new ProfiledPIDController(kPXController,0,0,new Constraints(4, 4));

    
    addRequirements(m_drivetrain, m_limelight);

    thetaController.setGoal(0);
    thetaController.setTolerance(Units.degreesToRadians(0),0);
    xController.setGoal(0);
    xController.setTolerance(Units.degreesToRadians(0),0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setNoteSeekerPipeline();
    m_limelight.setLED(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_limelight.isTargetVisible()) {

      double distance = -Math.abs(m_limelight.getDistanceToGoalMeters());
      m_drivetrain.drive(
        xController.calculate(-distance),
        0, 
        thetaController.calculate(m_limelight.getXOffsetRadians()), 
        false,
        false);
    }
    else {
      m_drivetrain.drive(
        0,
        0, 
        0, 
        false,
        false);
    }
  }  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limelight.setLED(false);
    m_drivetrain.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return thetaController.atGoal();
  }
}