// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private CANSparkFlex frontRollerMotor;
  private CANSparkFlex backRollerMotor;
  
  public Intake() {
    frontRollerMotor = new CANSparkFlex(IntakeConstants.kFrontRollerCanId, MotorType.kBrushless);
    backRollerMotor = new CANSparkFlex(IntakeConstants.kBackRollerCanId, MotorType.kBrushless);

    frontRollerMotor.setIdleMode(IdleMode.kBrake);
    backRollerMotor.setIdleMode(IdleMode.kBrake);

    frontRollerMotor.burnFlash();
    backRollerMotor.burnFlash();
    
  }

  public void frontIntake() {
    frontRollerMotor.setVoltage(IntakeConstants.kRollerVoltage);
  }

  public void frontOuttake() {
    frontRollerMotor.setVoltage(-1 * IntakeConstants.kRollerVoltage);
  }

  public void stopFront() {
    frontRollerMotor.setVoltage(0);
  }

   public void backIntake() {
    backRollerMotor.setVoltage(IntakeConstants.kRollerVoltage);
  }

  public void backOuttake() {
    backRollerMotor.setVoltage(-1 * IntakeConstants.kRollerVoltage);
  }

  public void stopBack() {
    backRollerMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
