// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private CANSparkFlex leftClimberMotor, rightClimberMotor;
  private RelativeEncoder leftClimberEncoder, rightClimberEncoder;

  public Climber() {

    leftClimberMotor = new CANSparkFlex(ClimberConstants.kLeftClimberCanId, MotorType.kBrushless);
    rightClimberMotor = new CANSparkFlex(ClimberConstants.kRightClimberCanId, MotorType.kBrushless);

    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);

    leftClimberMotor.setSmartCurrentLimit(ClimberConstants.kLeftClimberSmartCurrentLimit);
    rightClimberMotor.setSmartCurrentLimit(ClimberConstants.kRightClimberSmartCurrentLimit);


    leftClimberEncoder = leftClimberMotor.getEncoder();
    rightClimberEncoder = rightClimberMotor.getEncoder();

    leftClimberMotor.burnFlash();
    rightClimberMotor.burnFlash();
  }

  public void extendClimbers() {
    if (leftClimberEncoder.getPosition() < ClimberConstants.kMaxExtension) {
      leftClimberMotor.setVoltage(ClimberConstants.kClimberVoltage);
    }
    else {
      leftClimberMotor.setVoltage(0);
    }

    if (rightClimberEncoder.getPosition() < ClimberConstants.kMaxExtension) {
      rightClimberMotor.setVoltage(ClimberConstants.kClimberVoltage);
    }
    else {
      rightClimberMotor.setVoltage(0);
    }
  }

  public void retractClimbers() {
    if (leftClimberEncoder.getPosition() > ClimberConstants.kMinExtension) {
      leftClimberMotor.setVoltage(-ClimberConstants.kClimberVoltage);
    }
    else {
      leftClimberMotor.setVoltage(0);
    }

    if (rightClimberEncoder.getPosition() > ClimberConstants.kMinExtension) {
      rightClimberMotor.setVoltage(-ClimberConstants.kClimberVoltage);
    }
    else {
      rightClimberMotor.setVoltage(0);
    }
  }

  public double getLeftClimberPosition() {
    return leftClimberEncoder.getPosition();
  }

  public double getRightClimberPosition() {
    return rightClimberEncoder.getPosition();
  }

  public Command extendClimbersCommand() {
    return new InstantCommand(() -> extendClimbers());
  }

  public Command retractClimbersCommand() {
    return new InstantCommand(() -> retractClimbers());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climber Extension", leftClimberEncoder.getPosition());
    SmartDashboard.putNumber("Right Climber Extension", rightClimberEncoder.getPosition());
  }
}
