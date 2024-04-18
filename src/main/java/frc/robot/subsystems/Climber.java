// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utils.TunableNumber;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private CANSparkFlex leftClimberMotor, rightClimberMotor;
  private RelativeEncoder leftClimberEncoder, rightClimberEncoder;

  private PIDController leftClimberController;
  private PIDController rightClimberController;

  private TunableNumber climberP;

  private boolean configuring;

  public Climber() {

    leftClimberMotor = new CANSparkFlex(ClimberConstants.kLeftClimberCanId, MotorType.kBrushless);
    rightClimberMotor = new CANSparkFlex(ClimberConstants.kRightClimberCanId, MotorType.kBrushless);

    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);

    leftClimberMotor.setSmartCurrentLimit(ClimberConstants.kLeftClimberSmartCurrentLimit);
    rightClimberMotor.setSmartCurrentLimit(ClimberConstants.kRightClimberSmartCurrentLimit);

    leftClimberMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 20000);
    rightClimberMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 20000);

    leftClimberMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20000);
    rightClimberMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20000);

    leftClimberMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20000);
    rightClimberMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20000);

    leftClimberController = new PIDController(ClimberConstants.kP, 0, 0);
    rightClimberController = new PIDController(ClimberConstants.kP, 0, 0);

    climberP = new TunableNumber("Climber P");
    climberP.setDefault(0);


    leftClimberEncoder = leftClimberMotor.getEncoder();
    rightClimberEncoder = rightClimberMotor.getEncoder();

    leftClimberMotor.burnFlash();
    rightClimberMotor.burnFlash();

    configuring = false;
  }

  public Command enableConfiguring() {
    return new InstantCommand(() -> configuring = true);
  }

  public Command disableConfiguring() {
    return new InstantCommand(() -> configuring = false);
  }

  public void extendLeftClimber() {
    if (leftClimberEncoder.getPosition() < ClimberConstants.kMaxExtension) {
      leftClimberMotor.setVoltage(ClimberConstants.kClimberVoltage);
    }
    else {
      leftClimberMotor.setVoltage(0);
    }
  }

  public Command extendLeftClimberCommand() {
    return new InstantCommand(() -> extendLeftClimber());
  }

  public Command extendRightClimberCommand() {
    return new InstantCommand(() -> extendRightClimber());
  }

  public void extendRightClimber() {
    if (rightClimberEncoder.getPosition() < ClimberConstants.kMaxExtension) {
      rightClimberMotor.setVoltage(ClimberConstants.kClimberVoltage);
    }
    else {
      rightClimberMotor.setVoltage(0);
    }
  }

  public void retractLeftClimber() {
    if (leftClimberEncoder.getPosition() > ClimberConstants.kMinExtension) {
      leftClimberMotor.setVoltage(-ClimberConstants.kClimberVoltage);
    }
    else {
      leftClimberMotor.setVoltage(0);
    }
  }

  public Command retractLeftClimberCommand() {
    return new InstantCommand(() -> retractLeftClimber());
  }

  public Command retractRightClimberCommand() {
    return new InstantCommand(() -> retractRightClimber());
  }

  public void retractRightClimber() {
    if (rightClimberEncoder.getPosition() > ClimberConstants.kMinExtension) {
      rightClimberMotor.setVoltage(-ClimberConstants.kClimberVoltage);
    }
    else {
      rightClimberMotor.setVoltage(0);
    }
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

  public boolean leftClimberAtMax() {
    return leftClimberEncoder.getPosition() >= ClimberConstants.kMaxExtension;
  }

  public boolean rightClimberAtMax() {
    return rightClimberEncoder.getPosition() >= ClimberConstants.kMaxExtension;
  }

  public boolean leftClimberAtMin() {
    return leftClimberEncoder.getPosition() <= ClimberConstants.kMinExtension;
  }

  public boolean rightClimberAtMin() {
    return rightClimberEncoder.getPosition() <= ClimberConstants.kMinExtension;
  }

  public double getLeftClimberPosition() {
    return leftClimberEncoder.getPosition();
  }

  public double getRightClimberPosition() {
    return rightClimberEncoder.getPosition();
  }

  public Command extendLeftClimberSetpoint() {
    return new InstantCommand(() -> leftClimberController.setSetpoint(ClimberConstants.kMaxExtension));
  }

  public Command retractLeftClimberSetpoint() {
    return new InstantCommand(() -> leftClimberController.setSetpoint(ClimberConstants.kMinExtension));
  }

  public Command zeroLeftClimberSetpoint() {
    return new InstantCommand(() -> leftClimberController.setSetpoint(0));
  }

  public Command zeroRightClimberSetpoint() {
    return new InstantCommand(() -> rightClimberController.setSetpoint(0));
  }

  public Command extendRightClimberSetpoint() {
    return new InstantCommand(() -> rightClimberController.setSetpoint(ClimberConstants.kMaxExtension));
  }

  public Command retractRightClimberSetpoint() {
    return new InstantCommand(() -> rightClimberController.setSetpoint(ClimberConstants.kMinExtension));
  }

  public Command extendClimbersCommand() {
    return new SequentialCommandGroup(disableConfiguring(),
    new ParallelCommandGroup(extendLeftClimberSetpoint(), extendRightClimberSetpoint()));
  }

  public Command retractClimbersCommand() {
    return new SequentialCommandGroup(disableConfiguring(),
    new ParallelCommandGroup(retractLeftClimberSetpoint(), retractRightClimberSetpoint()));
  }

  public Command zeroClimbersCommand() {
    return new SequentialCommandGroup(disableConfiguring(),
    new ParallelCommandGroup(zeroLeftClimberSetpoint(), zeroRightClimberSetpoint()));
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

  public Command extendLeftClimberToReset() {
    
    return new SequentialCommandGroup(enableConfiguring(),
    new InstantCommand(() -> leftClimberMotor.setVoltage(ClimberConstants.kClimberConfigureVoltage)));
  }

  public Command stopLeftClimber() {
    return new InstantCommand(() -> leftClimberMotor.setVoltage(0));
  }

  public Command retractLeftClimberToReset() {
    return new SequentialCommandGroup(enableConfiguring(),
    new InstantCommand(() -> leftClimberMotor.setVoltage(-ClimberConstants.kClimberConfigureVoltage)));
  }

  public Command extendRightClimberToReset() {
    return new SequentialCommandGroup(enableConfiguring(),
    new InstantCommand(() -> rightClimberMotor.setVoltage(ClimberConstants.kClimberConfigureVoltage)));
  }

  public Command retractRightClimberToReset() {
    return new SequentialCommandGroup(enableConfiguring(),
    new InstantCommand(() -> rightClimberMotor.setVoltage(-ClimberConstants.kClimberConfigureVoltage)));
  }

  public Command stopRightClimber() {
    return new InstantCommand(() -> rightClimberMotor.setVoltage(0));
  }

  public void setCalculatedClimberVoltage() {
    leftClimberMotor.setVoltage(leftClimberController.calculate(getLeftClimberPosition()));
    rightClimberMotor.setVoltage(rightClimberController.calculate(getRightClimberPosition()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climber Extension", leftClimberEncoder.getPosition());
    SmartDashboard.putNumber("Right Climber Extension", rightClimberEncoder.getPosition());

    if (!configuring) setCalculatedClimberVoltage();
  }
}
