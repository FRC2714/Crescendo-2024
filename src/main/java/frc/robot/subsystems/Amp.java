// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.AmpConstants.AmpPIDConstants;
import frc.robot.Constants.ShooterConstants;

public class Amp extends SubsystemBase {
  /** Creates a new Amp. */

  private CANSparkFlex pivotMotor;
  private AbsoluteEncoder pivotEncoder;
  private PIDController pivotController;

  public Amp() {

    pivotMotor = new CANSparkFlex(AmpConstants.kAmpCanId, MotorType.kBrushless);
    pivotMotor.setIdleMode(IdleMode.kBrake);

    pivotMotor.setSmartCurrentLimit(AmpConstants.kSmartCurrentLimit);

    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pivotEncoder.setPositionConversionFactor(AmpConstants.kPivotConversionFactor);
    pivotEncoder.setZeroOffset(AmpConstants.kPivotZeroOffset);
  
    pivotController = new PIDController(AmpPIDConstants.kP, AmpPIDConstants.kI, AmpPIDConstants.kD);
    
    pivotMotor.burnFlash();
  }

  public double getPivotAngle() {
    return ((pivotEncoder.getPosition() - AmpConstants.kPivotEncoderKinematicOffset) / AmpConstants.kPivotGearRatio);
  }

  public double getTargetPivotAngle() {
    return pivotController.getSetpoint();
  }

  public void setPivotAngle(double targetAngle) {
    if (targetAngle > AmpConstants.kMaxPivotAngle)
      pivotController.setSetpoint(AmpConstants.kMaxPivotAngle);
    else if (targetAngle < AmpConstants.kMinPivotAngle)
      pivotController.setSetpoint(AmpConstants.kMinPivotAngle);
    else
      pivotController.setSetpoint(targetAngle);
  }

  public Command deploy() {
    return new InstantCommand(() -> setPivotAngle(AmpConstants.kDeployAngle));
  }

  public Command stow() {
    return new InstantCommand(() -> setPivotAngle(AmpConstants.kStowAngle));
  }

  public void setCalculatedPivotVoltage() {
    pivotMotor.setVoltage(pivotController.calculate(getPivotAngle()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Amp Angle", getPivotAngle());
    setCalculatedPivotVoltage();
  }
}
