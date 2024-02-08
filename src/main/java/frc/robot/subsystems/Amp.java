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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.AmpConstants.PivotPIDConstants;
import frc.robot.Constants.AmpConstants.RollerPIDConstants;

public class Amp extends SubsystemBase {
  /** Creates a new Amp. */

  private CANSparkFlex pivotMotor, rollerMotor;
  private AbsoluteEncoder pivotEncoder;
  private RelativeEncoder rollerEncoder;

  private PIDController pivotController, rollerController;
  private SimpleMotorFeedforward rollerFeedforward;

  public Amp() {

    pivotMotor = new CANSparkFlex(AmpConstants.kPivotCanId, MotorType.kBrushless);
    rollerMotor = new CANSparkFlex(AmpConstants.kRollerCanId, MotorType.kBrushless);

    pivotMotor.setIdleMode(IdleMode.kBrake);
    rollerMotor.setIdleMode(IdleMode.kCoast);

    pivotMotor.setSmartCurrentLimit(AmpConstants.kPivotSmartCurrentLimit);
    rollerMotor.setSmartCurrentLimit(AmpConstants.kRollerSmartCurrentLimit);

    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(AmpConstants.kPivotEncoderConversionFactor);

    rollerEncoder = rollerMotor.getEncoder();

    pivotController = new PIDController(PivotPIDConstants.kP, PivotPIDConstants.kI, PivotPIDConstants.kD);
    rollerController = new PIDController(RollerPIDConstants.kP, RollerPIDConstants.kI, RollerPIDConstants.kD);
    rollerFeedforward = new SimpleMotorFeedforward(RollerPIDConstants.kS, RollerPIDConstants.kV, RollerPIDConstants.kA);

    pivotMotor.burnFlash();
    rollerMotor.burnFlash();

  }

  public double getPivotAngle() {
    return pivotEncoder.getPosition() / AmpConstants.kPivotGearRatio;
  }

  public double getTargetPivotAngle() {
    return pivotController.getSetpoint();
  }

  public void setPivotAngle(double targetAngle) {
    pivotController.setSetpoint(targetAngle);
  }

  public double getRollerVelocity() {
    return rollerEncoder.getVelocity();
  }

  public double getTargetRollerVelocity() {
    return rollerController.getSetpoint();
  }

  public void setRollerVelocity(double targetVelocity) {
    rollerController.setSetpoint(targetVelocity);
  }

  public void setCalculatedPivotVoltage() {
    pivotMotor.setVoltage(pivotController.calculate(getPivotAngle()));
  }

  public void setCalculatedRollerVoltage() {
    rollerMotor.setVoltage(rollerController.calculate(getRollerVelocity()) + rollerFeedforward.calculate(getTargetRollerVelocity()));
  }

  public Command setRollerVelocityCommand(double targetVelocity) {
    return new InstantCommand(() -> setRollerVelocity(targetVelocity));
  }

  public Command setPivotAngleCommand(double targetVelocity) {
    return new InstantCommand(() -> setPivotAngle(targetVelocity));
  }

  public ParallelCommandGroup setStowPosition() {
    return new ParallelCommandGroup(setPivotAngleCommand(AmpConstants.kPivotStowAngle),
                                    setRollerVelocityCommand(0));
  }

  public ParallelCommandGroup setScorePosition() {
    return new ParallelCommandGroup(setPivotAngleCommand(AmpConstants.kPivotScoreAngle),
                                    setRollerVelocityCommand(AmpConstants.kRollerScoreVelocity));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current Pivot Angle", getPivotAngle());
    SmartDashboard.putNumber("Target Pivot Angle", getTargetPivotAngle());
    SmartDashboard.putNumber("Current Roller Velocity", getRollerVelocity());
    SmartDashboard.putNumber("Target Roller Velocity", getTargetRollerVelocity());

    setCalculatedPivotVoltage();
    setCalculatedRollerVoltage();
  }
}
