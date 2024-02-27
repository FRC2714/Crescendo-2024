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
  
  public void stopClimbers() {
    leftClimberMotor.setVoltage(0);
    rightClimberMotor.setVoltage(0);
  }

  public double getLeftClimberPosition() {
    return leftClimberEncoder.getPosition();
  }

  public double getRightClimberPosition() {
    return rightClimberEncoder.getPosition();
  }

  public boolean leftClimberAtMax() {
    return (leftClimberEncoder.getPosition() >= ClimberConstants.kMaxExtension);
  }

  public boolean rightClimberAtMax() {
    return (rightClimberEncoder.getPosition() >= ClimberConstants.kMaxExtension);
  }

  public boolean leftClimberAtMin() {
    return (leftClimberEncoder.getPosition() <= ClimberConstants.kMinExtension);
  }

  public boolean rightClimberAtMin() {
    return (rightClimberEncoder.getPosition() <= ClimberConstants.kMinExtension);
  }

  public Command extendClimbersCommand() {
    return new InstantCommand(() -> extendClimbers());
  }

  public Command retractClimbersCommand() {
    return new InstantCommand(() -> retractClimbers());
  }

  public Command stopClimbersCommand() {
    return new InstantCommand(() -> stopClimbers());
  }

  public Command setLeftClimberZero() {
    return new InstantCommand(() -> leftClimberEncoder.setPosition(0));
  }
  public Command setRightClimberZero() {
    return new InstantCommand(() -> rightClimberEncoder.setPosition(0));
  }

  public Command extendLeftClimber() {
    return new InstantCommand(() -> leftClimberMotor.setVoltage(ClimberConstants.kClimberVoltage));
  }

  public Command stopLeftClimber() {
    return new InstantCommand(() -> leftClimberMotor.setVoltage(0));
  }

  public Command retractLeftClimber() {
    return new InstantCommand(() -> leftClimberMotor.setVoltage(-ClimberConstants.kClimberVoltage));
  }

  public Command extendRightClimber() {
    return new InstantCommand(() -> rightClimberMotor.setVoltage(ClimberConstants.kClimberVoltage));
  }

  public Command retractRightClimber() {
    return new InstantCommand(() -> rightClimberMotor.setVoltage(-ClimberConstants.kClimberVoltage));
  }

  public Command stopRightClimber() {
    return new InstantCommand(() -> rightClimberMotor.setVoltage(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climber Extension", leftClimberEncoder.getPosition());
    SmartDashboard.putNumber("Right Climber Extension", rightClimberEncoder.getPosition());
  }
}
