// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelPIDConstants;
import frc.robot.Constants.ShooterConstants.PivotPIDConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private CANSparkFlex topFlywheelMotor;
  private CANSparkFlex bottomFlywheelMotor;
  private CANSparkFlex pivotMotor;

  private AbsoluteEncoder pivotEncoder;
  private RelativeEncoder flywheelEncoder;

  private PIDController pivotController;
  private PIDController flywheelController;
  private SimpleMotorFeedforward flywheelFeedforward;


  public Shooter() {
    topFlywheelMotor = new CANSparkFlex(ShooterConstants.kTopFlywheelCanId, MotorType.kBrushless);
    bottomFlywheelMotor = new CANSparkFlex(ShooterConstants.kTopFlywheelCanId, MotorType.kBrushless);
    pivotMotor = new CANSparkFlex(ShooterConstants.kBottomFlywheelCanId, MotorType.kBrushless);

    pivotMotor.setIdleMode(IdleMode.kBrake);
    topFlywheelMotor.setIdleMode(IdleMode.kBrake);
    bottomFlywheelMotor.setIdleMode(IdleMode.kBrake);

    pivotMotor.setSmartCurrentLimit(ShooterConstants.kPivotSmartCurrentLimit);
    topFlywheelMotor.setSmartCurrentLimit(ShooterConstants.kTopFlywheelSmartCurrentLimit);
    bottomFlywheelMotor.setSmartCurrentLimit(ShooterConstants.kBottomFlywheelSmartCurrentLimit);

    bottomFlywheelMotor.follow(topFlywheelMotor, true);

    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(ShooterConstants.kPivotEncoderConversionFactor);

    flywheelEncoder = topFlywheelMotor.getEncoder();

    pivotController = new PIDController(PivotPIDConstants.kP, PivotPIDConstants.kI, PivotPIDConstants.kD);
    flywheelController = new PIDController(FlywheelPIDConstants.kP, FlywheelPIDConstants.kI, FlywheelPIDConstants.kD);
    flywheelFeedforward = new SimpleMotorFeedforward(FlywheelPIDConstants.kS, FlywheelPIDConstants.kV, FlywheelPIDConstants.kA);

    topFlywheelMotor.burnFlash();
    bottomFlywheelMotor.burnFlash();
    pivotMotor.burnFlash();
  }

  public double getPivotAngle() {
    return pivotEncoder.getPosition() / ShooterConstants.kPivotGearRatio;
  }

  public double getTargetPivotAngle() {
    return pivotController.getSetpoint();
  }

  public void setPivotAngle(double targetAngle) {
    pivotController.setSetpoint(targetAngle);
  }

  public double getFlywheelVelocity() {
    return flywheelEncoder.getVelocity();
  }

  public double getTargetFlywheelVelocity() {
    return flywheelController.getSetpoint();
  }

  public void setFlywheelVelocity(double targetVelocity) {
    flywheelController.setSetpoint(targetVelocity);
  }

  public void setCalculatedPivotVoltage() {
    pivotMotor.setVoltage(pivotController.calculate(getPivotAngle()));
  }

  public void setCalculatedFlywheelVoltage() {
    pivotMotor.setVoltage(flywheelController.calculate(getFlywheelVelocity()) + flywheelFeedforward.calculate(getTargetFlywheelVelocity()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current Pivot Angle", getPivotAngle());
    SmartDashboard.putNumber("Target Pivot Angle", getTargetPivotAngle());
    SmartDashboard.putNumber("Current Flywheel Velocity", getFlywheelVelocity());
    SmartDashboard.putNumber("Target Flywheel Velocity", getTargetFlywheelVelocity());

    setCalculatedPivotVoltage();
    setCalculatedFlywheelVoltage();
  }
}
