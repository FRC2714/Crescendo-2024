// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants.ThetaPIDConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RotateToGoal extends Command {
  
  private DriveSubsystem m_drivetrain;
  private Vision m_camera;

  private PIDController thetaController;

  /** Creates a new RotateToGoal. */
  public RotateToGoal(DriveSubsystem m_drivetrain, Vision m_camera) {
    this.m_drivetrain = m_drivetrain;
    this.m_camera = m_camera;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain, m_camera);

    thetaController = new PIDController(ThetaPIDConstants.kP, ThetaPIDConstants.kI, ThetaPIDConstants.kD);
    // thetaController = new ProfiledPIDController(ThetaPIDConstants.kP, ThetaPIDConstants.kI, ThetaPIDConstants.kD, )

    // thetaController.setSetpoint(Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(0),0);
    thetaController.enableContinuousInput(0, 2 *  Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      PhotonTrackedTarget speakerTarget = m_camera.getSpeakerTarget();
      
      if(speakerTarget != null){
      double angleToSpeaker =  -speakerTarget.getBestCameraToTarget().getRotation().getZ();
      double angleToTurn = angleToSpeaker > 0 ? angleToSpeaker: (Math.PI * 2) + angleToSpeaker;
      // double setpoint =  angleToSpeaker > 0 ?
      //                             Math.atan(Units.inchesToMeters(11)/speakerTarget.getBestCameraToTarget().getX()):
      //                             -Math.atan(Units.inchesToMeters(11)/speakerTarget.getBestCameraToTarget().getX());
    

      double rotationSpeed = thetaController.calculate(angleToTurn, Math.PI);
      if(rotationSpeed < -0.5)
      {
        rotationSpeed = -0.5;
      }
      else if(rotationSpeed > 0.5)
      {
        rotationSpeed = 0.5;
      }
    m_drivetrain.drive(
      0, 
      0, 
      (rotationSpeed),
      true,
      false);
    } else {
      m_drivetrain.drive(
        0, 
        0, 
        0,
        true,
        false);
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
    return thetaController.atSetpoint();
  }
}