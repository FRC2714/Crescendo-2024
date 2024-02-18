// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private CANSparkFlex frontRollerMotor;
  private CANSparkFlex backBottomRollerMotor;
  private CANSparkFlex backDirectionRollerMotor;
  private CANSparkFlex conveyorMotor;
  private CANSparkFlex feederMotor;

  private RelativeEncoder frontRollerEncoder;
  private RelativeEncoder backBottomRollerEncoder;
  private RelativeEncoder backDirectionRollerEncoder;
  private RelativeEncoder conveyorEncoder;
  private RelativeEncoder feederEncoder;

  private boolean backRunning, frontRunning;
  
  public Intake() {
    frontRollerMotor = new CANSparkFlex(IntakeConstants.kFrontRollerCanId, MotorType.kBrushless);
    backBottomRollerMotor = new CANSparkFlex(IntakeConstants.kBackBottomRollerCanId, MotorType.kBrushless);
    backDirectionRollerMotor = new CANSparkFlex(IntakeConstants.kBackDirectionRollerCanId, MotorType.kBrushless);
    conveyorMotor = new CANSparkFlex(IntakeConstants.kConveyorCanId, MotorType.kBrushless);
    feederMotor = new CANSparkFlex(IntakeConstants.kFeederCanId, MotorType.kBrushless);

    frontRollerMotor.setInverted(true);

    frontRollerMotor.setIdleMode(IdleMode.kBrake);
    backBottomRollerMotor.setIdleMode(IdleMode.kBrake);
    backDirectionRollerMotor.setIdleMode(IdleMode.kBrake);
    conveyorMotor.setIdleMode(IdleMode.kBrake);
    feederMotor.setIdleMode(IdleMode.kBrake);

    frontRollerMotor.enableVoltageCompensation(IntakeConstants.kRollerNominalVoltage);
    backBottomRollerMotor.enableVoltageCompensation(IntakeConstants.kRollerNominalVoltage);
    backDirectionRollerMotor.enableVoltageCompensation(IntakeConstants.kRollerNominalVoltage);
    conveyorMotor.enableVoltageCompensation(IntakeConstants.kRollerNominalVoltage);
    feederMotor.enableVoltageCompensation(IntakeConstants.kRollerNominalVoltage);

    // frontRollerMotor.setSmartCurrentLimit(IntakeConstants.kFrontRollerSmartCurrentLimit);
    // backBottomRollerMotor.setSmartCurrentLimit(IntakeConstants.kBackBottomRollerSmartCurrentLimit);
    // backDirectionRollerMotor.setSmartCurrentLimit(IntakeConstants.kBackDirectionRollerSmartCurrentLimit);
    // conveyorMotor.setSmartCurrentLimit(IntakeConstants.kConveyorSmartCurrentLimit);

    frontRollerEncoder = frontRollerMotor.getEncoder();
    backBottomRollerEncoder = backBottomRollerMotor.getEncoder();
    backDirectionRollerEncoder = backDirectionRollerMotor.getEncoder();
    conveyorEncoder = conveyorMotor.getEncoder();
    feederEncoder = feederMotor.getEncoder();

    frontRollerMotor.burnFlash();
    backBottomRollerMotor.burnFlash();
    backDirectionRollerMotor.burnFlash();
    conveyorMotor.burnFlash();
    feederMotor.burnFlash();

    backRunning = false;
    frontRunning = false;
    
  }

  public double getFrontRollerVelocity() {
    return frontRollerEncoder.getVelocity();
  }

  public double getBackBottomRollerVelocity() {
    return backBottomRollerEncoder.getVelocity();
  }

  public double getBackDirectionRollerVelocity() {
    return backDirectionRollerEncoder.getVelocity();
  }

  public double getConveyorVelocity() {
    return conveyorEncoder.getVelocity();
  }

  public double getFeederVelocity() {
    return feederEncoder.getVelocity();
  }

  public void setFrontRollerVoltage(double targetVoltage) {
    frontRollerMotor.setVoltage(targetVoltage);
  }

  public void setBackBottomRollerVoltage(double targetVoltage) {
    backBottomRollerMotor.setVoltage(targetVoltage);
  }

  public void setBackDirectionRollerVoltage(double targetVoltage) {
    backDirectionRollerMotor.setVoltage(targetVoltage);
  }

  public void setConveyorVoltage(double targetVoltage) {
    conveyorMotor.setVoltage(targetVoltage);
  }

  public void setFeederVoltage(double targetVoltage) {
    feederMotor.setVoltage(targetVoltage);
  }

  public Command setFrontRollerVoltageCommand(double targetVoltage) {
    return new InstantCommand(() -> setFrontRollerVoltage(targetVoltage));
  }

  public Command setBackBottomRollerVoltageCommand(double targetVoltage) {
    return new InstantCommand(() -> setBackBottomRollerVoltage(targetVoltage));
  }

  public Command setBackDirectionRollerVoltageCommand(double targetVoltage) {
    return new InstantCommand(() -> setBackDirectionRollerVoltage(targetVoltage));
  }

  public Command setConveyorVoltageCommand(double targetVoltage) {
    return new InstantCommand(() -> setConveyorVoltage(targetVoltage));
  }

  public Command setFeederVoltageCommand(double targetVoltage) {
    return new InstantCommand(() -> setFeederVoltage(targetVoltage));
  }

  public ParallelCommandGroup intakeFront() {
    if (backRunning) return new ParallelCommandGroup();
    else frontRunning = true;
    return new ParallelCommandGroup(setFrontRollerVoltageCommand(IntakeConstants.kFrontRollerVoltage),
                                    setBackBottomRollerVoltageCommand(IntakeConstants.kBackBottomRollerVoltageFrontSide),
                                    setBackDirectionRollerVoltageCommand(IntakeConstants.kBackDirectionRollerVoltageFrontSide),
                                    setConveyorVoltageCommand(-IntakeConstants.kConveyorVoltage),
                                    setFeederVoltageCommand(IntakeConstants.kFeederVoltage));
  }

  public ParallelCommandGroup outtakeFront() {
    if (backRunning) return new ParallelCommandGroup();
    else frontRunning = true;
    return new ParallelCommandGroup(setFrontRollerVoltageCommand(-(IntakeConstants.kFrontRollerVoltage + 1)),
                                    setBackBottomRollerVoltageCommand(-(IntakeConstants.kBackBottomRollerVoltageFrontSide + 1)),
                                    setBackDirectionRollerVoltageCommand(-(IntakeConstants.kBackDirectionRollerVoltageFrontSide + 1)),
                                    setConveyorVoltageCommand(IntakeConstants.kConveyorVoltage + 1),
                                    setFeederVoltageCommand(-(IntakeConstants.kFeederVoltage + 1)));
  }

  public ParallelCommandGroup stopFront() {
    frontRunning = false;
    return new ParallelCommandGroup(setFrontRollerVoltageCommand(0),
                                    setBackBottomRollerVoltageCommand(0),
                                    setBackDirectionRollerVoltageCommand(0),
                                    setConveyorVoltageCommand(0),
                                    setFeederVoltageCommand(0));
  }

  public ParallelCommandGroup intakeBack() {
    if (frontRunning) return new ParallelCommandGroup();
    else backRunning = true;
    return new ParallelCommandGroup(setBackBottomRollerVoltageCommand(-IntakeConstants.kBackBottomRollerVoltageBackSide),
                                    setBackDirectionRollerVoltageCommand(IntakeConstants.kBackDirectionRollerVoltageBackSide),
                                    setFeederVoltageCommand(IntakeConstants.kFeederVoltage));
  }

  public ParallelCommandGroup outtakeBack() {
    if (frontRunning) return new ParallelCommandGroup();
    else backRunning = true;
    return new ParallelCommandGroup(setBackBottomRollerVoltageCommand(IntakeConstants.kBackBottomRollerVoltageBackSide + 1),
                                    setBackDirectionRollerVoltageCommand(-(IntakeConstants.kBackDirectionRollerVoltageBackSide + 1)),
                                    setFeederVoltageCommand(-(IntakeConstants.kFeederVoltage + 1)));
  }

  public ParallelCommandGroup stopBack() {
    backRunning = false;
    return new ParallelCommandGroup(setBackBottomRollerVoltageCommand(0),
                                    setBackDirectionRollerVoltageCommand(0),
                                    setFeederVoltageCommand(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Front Roller Velocity", getFrontRollerVelocity());
    SmartDashboard.putNumber("Back Bottom Roller Velocity", getBackBottomRollerVelocity());
    SmartDashboard.putNumber("Back Direction Roller Velocity", getBackDirectionRollerVelocity());
    SmartDashboard.putNumber("Conveyor Velocity", getConveyorVelocity());
  }
<<<<<<< Updated upstream
}
=======
}
>>>>>>> Stashed changes
