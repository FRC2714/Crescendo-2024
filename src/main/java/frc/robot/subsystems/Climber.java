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
import frc.robot.Constants.ClimberConstants.LeftClimberPIDConstants;
import frc.robot.Constants.ClimberConstants.RightClimberPIDConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private CANSparkFlex leftClimberMotor, rightClimberMotor;
  private RelativeEncoder leftClimberEncoder, rightClimberEncoder;

  private ProfiledPIDController leftClimberController, rightClimberController;
  private ElevatorFeedforward leftClimberFeedforward, rightClimberFeedforward;

  public Climber() {

    leftClimberMotor = new CANSparkFlex(ClimberConstants.kLeftClimberCanId, MotorType.kBrushless);
    rightClimberMotor = new CANSparkFlex(ClimberConstants.kRightClimberCanId, MotorType.kBrushless);

    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);

    leftClimberMotor.setSmartCurrentLimit(ClimberConstants.kLeftClimberSmartCurrentLimit);
    rightClimberMotor.setSmartCurrentLimit(ClimberConstants.kRightClimberSmartCurrentLimit);


    leftClimberEncoder = leftClimberMotor.getEncoder();
    rightClimberEncoder = rightClimberMotor.getEncoder();

    leftClimberController = new ProfiledPIDController(LeftClimberPIDConstants.kP,
                                                      LeftClimberPIDConstants.kI,
                                                      LeftClimberPIDConstants.kD,
                                                      ClimberConstants.kLeftClimberConstraints);
    
    rightClimberController = new ProfiledPIDController(RightClimberPIDConstants.kP,
                                                       RightClimberPIDConstants.kI,
                                                       RightClimberPIDConstants.kD,
                                                       ClimberConstants.kRightClimberConstraints);
    leftClimberFeedforward = new ElevatorFeedforward(LeftClimberPIDConstants.kS,
                                                     LeftClimberPIDConstants.kG,
                                                     LeftClimberPIDConstants.kV,
                                                     LeftClimberPIDConstants.kA);

    rightClimberFeedforward = new ElevatorFeedforward(RightClimberPIDConstants.kS,
                                                     RightClimberPIDConstants.kG,
                                                     RightClimberPIDConstants.kV,
                                                     RightClimberPIDConstants.kA);

    leftClimberMotor.burnFlash();
    rightClimberMotor.burnFlash();
  }

  public void extendClimbers() {
    leftClimberController.setGoal(ClimberConstants.kGroundExtension);
    rightClimberController.setGoal(ClimberConstants.kGroundExtension);
  }

  public void retractClimbers() {
    leftClimberController.setGoal(ClimberConstants.kGroundRetraction);
    rightClimberController.setGoal(ClimberConstants.kGroundRetraction);
  }

  public void setLeftClimber(double target) {
    target = target > ClimberConstants.kMaxExtension ? ClimberConstants.kMaxExtension : target;
    target = target < ClimberConstants.kMinExtension ? ClimberConstants.kMinExtension : target;
    State goal = new State(target, 0);
    leftClimberController.setGoal(goal);
  }

  public void setRightClimber(double target) {
    target = target > ClimberConstants.kMaxExtension ? ClimberConstants.kMaxExtension : target;
    target = target < ClimberConstants.kMinExtension ? ClimberConstants.kMinExtension : target;
    State goal = new State(target, 0);
    rightClimberController.setGoal(goal);
  }

  public double getLeftClimberPosition() {
    return leftClimberEncoder.getPosition();
  }

  public double getRightClimberPosition() {
    return rightClimberEncoder.getPosition();
  }

  public boolean leftClimberAtMax() {
    return Math.abs(ClimberConstants.kMaxExtension - leftClimberEncoder.getPosition()) < ClimberConstants.kSetpointTolerance;
  }

  public boolean leftClimberAtMin() {
    return Math.abs(ClimberConstants.kMinExtension - rightClimberEncoder.getPosition()) < ClimberConstants.kSetpointTolerance;
  }

  public boolean rightClimberAtMax() {
    return Math.abs(ClimberConstants.kMaxExtension - rightClimberEncoder.getPosition()) < ClimberConstants.kSetpointTolerance;
  }

  public boolean rightClimberAtMin() {
    return Math.abs(ClimberConstants.kMinExtension - rightClimberEncoder.getPosition()) < ClimberConstants.kSetpointTolerance;
  }

  public void setCalculatedClimberVoltage() {
    leftClimberMotor.setVoltage(
      leftClimberController.calculate(getLeftClimberPosition()) +
      leftClimberFeedforward.calculate(leftClimberController.getSetpoint().velocity)
    );
    rightClimberMotor.setVoltage(
      rightClimberController.calculate(getRightClimberPosition()) +
      rightClimberFeedforward.calculate(rightClimberController.getSetpoint().velocity)
    );
  }

  public Command extendClimbersCommand() {
    return new InstantCommand(() -> extendClimbers());
  }

  public Command retractClimbersCommand() {
    return new InstantCommand(() -> retractClimbers());
  }

  public Command setLeftClimberCommand(double target) {
    return new InstantCommand(() -> setLeftClimber(target));
  }

  public Command setRightClimberCommand(double target) {
    return new InstantCommand(() -> setRightClimber(target));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Climber Extension", leftClimberEncoder.getPosition());
    SmartDashboard.putNumber("Right Climber Extension", rightClimberEncoder.getPosition());

    setCalculatedClimberVoltage();
  }
}
