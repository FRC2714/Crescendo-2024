// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.BackBottomRollerPIDConstants;
import frc.robot.Constants.IntakeConstants.BackDirectionRollerPIDConstants;
import frc.robot.Constants.IntakeConstants.ConveyorPIDConstants;
import frc.robot.Constants.IntakeConstants.FrontRollerPIDConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private CANSparkFlex frontRollerMotor;
  private CANSparkFlex backBottomRollerMotor;
  private CANSparkFlex backDirectionRollerMotor;
  private CANSparkFlex conveyorMotor;

  private RelativeEncoder frontRollerEncoder;
  private RelativeEncoder backBottomRollerEncoder;
  private RelativeEncoder backDirectionRollerEncoder;
  private RelativeEncoder conveyorEncoder;

  private PIDController frontRollerController;
  private PIDController backBottomRollerController;
  private PIDController backDirectionRollerController;
  private PIDController conveyorController;

  private SimpleMotorFeedforward frontRollerFeedforward;
  private SimpleMotorFeedforward backBottomRollerFeedforward;
  private SimpleMotorFeedforward backDirectionRollerFeedforward;
  private SimpleMotorFeedforward conveyorFeedforward;
  
  public Intake() {
    frontRollerMotor = new CANSparkFlex(IntakeConstants.kFrontRollerCanId, MotorType.kBrushless);
    backBottomRollerMotor = new CANSparkFlex(IntakeConstants.kBackBottomRollerCanId, MotorType.kBrushless);
    backDirectionRollerMotor = new CANSparkFlex(IntakeConstants.kBackDirectionRollerCanId, MotorType.kBrushless);
    conveyorMotor = new CANSparkFlex(IntakeConstants.kConveyorCanId, MotorType.kBrushless);

    frontRollerMotor.setIdleMode(IdleMode.kBrake);
    backBottomRollerMotor.setIdleMode(IdleMode.kBrake);
    backDirectionRollerMotor.setIdleMode(IdleMode.kBrake);
    conveyorMotor.setIdleMode(IdleMode.kBrake);

    frontRollerMotor.setSmartCurrentLimit(IntakeConstants.kFrontRollerSmartCurrentLimit);
    backBottomRollerMotor.setSmartCurrentLimit(IntakeConstants.kBackBottomRollerSmartCurrentLimit);
    backDirectionRollerMotor.setSmartCurrentLimit(IntakeConstants.kBackDirectionRollerSmartCurrentLimit);
    conveyorMotor.setSmartCurrentLimit(IntakeConstants.kConveyorSmartCurrentLimit);

    frontRollerEncoder = frontRollerMotor.getEncoder();
    backBottomRollerEncoder = backBottomRollerMotor.getEncoder();
    backDirectionRollerEncoder = backDirectionRollerMotor.getEncoder();
    conveyorEncoder = conveyorMotor.getEncoder();

    frontRollerController = new PIDController(FrontRollerPIDConstants.kP,
                                              FrontRollerPIDConstants.kI,
                                              FrontRollerPIDConstants.kD);
    backBottomRollerController = new PIDController(BackBottomRollerPIDConstants.kP,
                                                   BackBottomRollerPIDConstants.kI,
                                                   BackBottomRollerPIDConstants.kD);
    backDirectionRollerController = new PIDController(BackDirectionRollerPIDConstants.kS,
                                                      BackDirectionRollerPIDConstants.kV,
                                                      BackDirectionRollerPIDConstants.kA);
    conveyorController = new PIDController(ConveyorPIDConstants.kP,
                                           ConveyorPIDConstants.kI,
                                           ConveyorPIDConstants.kD);

    frontRollerFeedforward = new SimpleMotorFeedforward(FrontRollerPIDConstants.kS,
                                                        FrontRollerPIDConstants.kV,
                                                        FrontRollerPIDConstants.kA);

    backBottomRollerFeedforward = new SimpleMotorFeedforward(BackBottomRollerPIDConstants.kS,
                                                             BackBottomRollerPIDConstants.kV,
                                                             BackBottomRollerPIDConstants.kA);
    
    backDirectionRollerFeedforward = new SimpleMotorFeedforward(BackDirectionRollerPIDConstants.kS,
                                                                BackDirectionRollerPIDConstants.kV,
                                                                BackDirectionRollerPIDConstants.kA);
    conveyorFeedforward = new SimpleMotorFeedforward(ConveyorPIDConstants.kS,
                                                     ConveyorPIDConstants.kV,
                                                     ConveyorPIDConstants.kA);


    frontRollerMotor.burnFlash();
    backBottomRollerMotor.burnFlash();
    backDirectionRollerMotor.burnFlash();
    conveyorMotor.burnFlash();
    
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

  public double getTargetFrontRollerVelocity() {
    return frontRollerController.getSetpoint();
  }

  public double getTargetBackBottomRollerVelocity() {
    return backBottomRollerController.getSetpoint();
  }

  public double getTargetBackDirectionRollerVelocity() {
    return backDirectionRollerController.getSetpoint();
  }

  public double getTargetConveyorVelocity() {
    return conveyorController.getSetpoint();
  }

  public void setFrontRollerVelocity(double targetVelocity) {
    frontRollerController.setSetpoint(targetVelocity);
  }

  public void setBackBottomRollerVelocity(double targetVelocity) {
    backBottomRollerController.setSetpoint(targetVelocity);
  }

  public void setBackDirectionRollerVelocity(double targetVelocity) {
    backDirectionRollerController.setSetpoint(targetVelocity);
  }

  public void setConveyorVelocity(double targetVelocity) {
    conveyorController.setSetpoint(targetVelocity);
  }

  public Command setFrontRollerVelocityCommand(double targetVelocity) {
    return new InstantCommand(() -> setFrontRollerVelocity(targetVelocity));
  }

  public Command setBackBottomRollerVelocityCommand(double targetVelocity) {
    return new InstantCommand(() -> setBackBottomRollerVelocity(targetVelocity));
  }

  public Command setBackDirectionRollerVelocityCommand(double targetVelocity) {
    return new InstantCommand(() -> setBackDirectionRollerVelocity(targetVelocity));
  }

  public Command setConveyorVelocityCommand(double targetVelocity) {
    return new InstantCommand(() -> setConveyorVelocity(targetVelocity));
  }

  public void setCalculatedFrontRollerVoltage() {
    frontRollerMotor.setVoltage(frontRollerController.calculate(getFrontRollerVelocity()) +
                                frontRollerFeedforward.calculate(getTargetFrontRollerVelocity()));
  }

  public void setCalculatedBackBottomRollerVoltage() {
    backBottomRollerMotor.setVoltage(backBottomRollerController.calculate(getBackBottomRollerVelocity()) +
                                     backBottomRollerFeedforward.calculate(getTargetBackBottomRollerVelocity()));
  }

  public void setCalculatedBackDirectionRollerVoltage() {
    backDirectionRollerMotor.setVoltage(backDirectionRollerController.calculate(getBackDirectionRollerVelocity()) +
                                        backDirectionRollerFeedforward.calculate(getTargetBackDirectionRollerVelocity()));
  }

  public void setCalculatedConveyorVoltage() {
    conveyorMotor.setVoltage(conveyorController.calculate(getConveyorVelocity()) +
                             conveyorFeedforward.calculate(getTargetConveyorVelocity()));
  }

  public ParallelCommandGroup intakeFront() {
    return new ParallelCommandGroup(setFrontRollerVelocityCommand(IntakeConstants.kFrontRollerVelocity),
                                    setBackBottomRollerVelocityCommand(IntakeConstants.kBackDirectionRollerVelocityFrontSide),
                                    setBackDirectionRollerVelocityCommand(IntakeConstants.kBackDirectionRollerVelocityFrontSide),
                                    setConveyorVelocityCommand(IntakeConstants.kConveyorVelocity));
  }

  public ParallelCommandGroup outtakeFront() {
    return new ParallelCommandGroup(setFrontRollerVelocityCommand(-IntakeConstants.kFrontRollerVelocity),
                                    setBackBottomRollerVelocityCommand(-IntakeConstants.kBackDirectionRollerVelocityFrontSide),
                                    setBackDirectionRollerVelocityCommand(-IntakeConstants.kBackDirectionRollerVelocityFrontSide),
                                    setConveyorVelocityCommand(-IntakeConstants.kConveyorVelocity));
  }

  public ParallelCommandGroup stopFront() {
    return new ParallelCommandGroup(setFrontRollerVelocityCommand(0),
                                    setBackBottomRollerVelocityCommand(0),
                                    setBackDirectionRollerVelocityCommand(0),
                                    setConveyorVelocityCommand(0));
  }

  public ParallelCommandGroup intakeBack() {
    return new ParallelCommandGroup(setBackBottomRollerVelocityCommand(IntakeConstants.kBackDirectionRollerVelocityBackSide),
                                    setBackDirectionRollerVelocityCommand(IntakeConstants.kBackDirectionRollerVelocityBackSide));
  }

  public ParallelCommandGroup outtakeBack() {
    return new ParallelCommandGroup(setBackBottomRollerVelocityCommand(-IntakeConstants.kBackDirectionRollerVelocityBackSide),
                                    setBackDirectionRollerVelocityCommand(-IntakeConstants.kBackDirectionRollerVelocityBackSide));
  }

  public ParallelCommandGroup stopBack() {
    return new ParallelCommandGroup(setBackBottomRollerVelocityCommand(0),
                                    setBackDirectionRollerVelocityCommand(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Front Roller Velocity", getFrontRollerVelocity());
    SmartDashboard.putNumber("Back Bottom Roller Velocity", getBackBottomRollerVelocity());
    SmartDashboard.putNumber("Back Direction Roller Velocity", getBackDirectionRollerVelocity());
    SmartDashboard.putNumber("Conveyor Velocity", getConveyorVelocity());

    setCalculatedFrontRollerVoltage();
    setCalculatedBackBottomRollerVoltage();
    setCalculatedBackDirectionRollerVoltage();
    setCalculatedConveyorVoltage();
  }
}
