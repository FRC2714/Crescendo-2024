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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FeederPIDConstants;
import frc.robot.Constants.ShooterConstants.FlywheelPIDConstants;
import frc.robot.Constants.ShooterConstants.PivotPIDConstants;
import frc.robot.utils.InterpolatingTreeMap;
import frc.robot.utils.TunableNumber;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private CANSparkFlex topFlywheelMotor;
  private CANSparkFlex bottomFlywheelMotor;
  private CANSparkFlex pivotMotor;
  private CANSparkFlex feederMotor;

  private AbsoluteEncoder pivotEncoder;
  private RelativeEncoder flywheelEncoder;
  private RelativeEncoder feederEncoder;

  private PIDController pivotController;
  private PIDController flywheelController;
  private PIDController feederController;
  private SimpleMotorFeedforward flywheelFeedforward;

  private Limelight m_limelight;

  private InterpolatingTreeMap pivotAngleMap;
  private InterpolatingTreeMap flywheelVelocityMap;
  private InterpolatingTreeMap shootTimeMap;

  private TunableNumber pivotAngleTunableNumber;
  private TunableNumber flywheelVelocityTunableNumber;

  private boolean dynamicEnabled;


  public Shooter(Limelight m_limelight) {
    pivotMotor = new CANSparkFlex(ShooterConstants.kPivotCanId, MotorType.kBrushless);
    feederMotor = new CANSparkFlex(ShooterConstants.kFeederCanId, MotorType.kBrushless);
    topFlywheelMotor = new CANSparkFlex(ShooterConstants.kTopFlywheelCanId, MotorType.kBrushless);
    bottomFlywheelMotor = new CANSparkFlex(ShooterConstants.kBottomFlywheelCanId, MotorType.kBrushless);

    pivotMotor.setIdleMode(IdleMode.kBrake);
    feederMotor.setIdleMode(IdleMode.kCoast);
    topFlywheelMotor.setIdleMode(IdleMode.kCoast);
    bottomFlywheelMotor.setIdleMode(IdleMode.kCoast);

    pivotMotor.setSmartCurrentLimit(ShooterConstants.kPivotSmartCurrentLimit);
    feederMotor.setSmartCurrentLimit(ShooterConstants.kPivotSmartCurrentLimit);
    topFlywheelMotor.setSmartCurrentLimit(ShooterConstants.kTopFlywheelSmartCurrentLimit);
    bottomFlywheelMotor.setSmartCurrentLimit(ShooterConstants.kBottomFlywheelSmartCurrentLimit);

    bottomFlywheelMotor.follow(topFlywheelMotor, true);

    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(ShooterConstants.kPivotEncoderConversionFactor);

    flywheelEncoder = topFlywheelMotor.getEncoder();
    feederEncoder = feederMotor.getEncoder();

    topFlywheelMotor.enableVoltageCompensation(ShooterConstants.kNominalVoltage);
    bottomFlywheelMotor.enableVoltageCompensation(ShooterConstants.kNominalVoltage);

    pivotController = new PIDController(PivotPIDConstants.kP, PivotPIDConstants.kI, PivotPIDConstants.kD);
    feederController = new PIDController(FeederPIDConstants.kP, FeederPIDConstants.kI, FeederPIDConstants.kD);
    flywheelController = new PIDController(FlywheelPIDConstants.kP, FlywheelPIDConstants.kI, FlywheelPIDConstants.kD);
    flywheelFeedforward = new SimpleMotorFeedforward(FlywheelPIDConstants.kS, FlywheelPIDConstants.kV, FlywheelPIDConstants.kA);

    topFlywheelMotor.burnFlash();
    bottomFlywheelMotor.burnFlash();
    pivotMotor.burnFlash();
    feederMotor.burnFlash();

    pivotAngleMap = new InterpolatingTreeMap();
    flywheelVelocityMap = new InterpolatingTreeMap();
    shootTimeMap = new InterpolatingTreeMap();

    dynamicEnabled = false;
    
    populatePivotAngleMap();
    populateFlywheelVelocityMap();
    populateShootTimeMap();

    pivotAngleTunableNumber = new TunableNumber("Tunable Pivot Angle");
    flywheelVelocityTunableNumber = new TunableNumber("Tunable Flywheel Velocity");

    this.m_limelight = m_limelight;
  }

  public void toggleDynamic() {
    dynamicEnabled = !dynamicEnabled;
  }

  public void populatePivotAngleMap() {
    pivotAngleMap.put(0.0, 0.0); // TBD
  }

  public void populateFlywheelVelocityMap() {
    flywheelVelocityMap.put(0.0, 0.0); // TBD
  }

  public void populateShootTimeMap() {
    shootTimeMap.put(0.0, 0.0); // TBD
  }

  public double getInterpolatedShootTime(double distance) {
    return shootTimeMap.getInterpolated(distance);
  }

  public double getPivotAngle() {
    return pivotEncoder.getPosition() / ShooterConstants.kPivotGearRatio;
  }

  public double getTargetPivotAngle() {
    return pivotController.getSetpoint();
  }

  public void setPivotAngle(double targetAngle) {
    if (!dynamicEnabled) pivotController.setSetpoint(targetAngle);
  }

  public double getFlywheelVelocity() {
    return flywheelEncoder.getVelocity();
  }

  public double getTargetFlywheelVelocity() {
    return flywheelController.getSetpoint();
  }

  public double getFeederVelocity() {
    return feederEncoder.getVelocity();
  }

  public double getTargetFeederVelocity() {
    return feederController.getSetpoint();
  }

  public void setFeederVelocity(double targetVelocity) {
    feederController.setSetpoint(targetVelocity);
  }

  public void setFlywheelVelocity(double targetVelocity) {
    if (!dynamicEnabled) flywheelController.setSetpoint(targetVelocity);
  }

  public double getDynamicPivotAngle() {
    return m_limelight.isTargetVisible()
      ? pivotAngleMap.getInterpolated(m_limelight.getDistanceToGoalMeters())
      : 0;
  }

  public double getDynamicFlywheelVelocity() {
    return m_limelight.isTargetVisible()
      ? flywheelVelocityMap.getInterpolated(m_limelight.getDistanceToGoalMeters())
      : 0;
  }

  public double getDynamicPivotAngle(double adjustedDistance) {
    return m_limelight.isTargetVisible()
      ? pivotAngleMap.getInterpolated(adjustedDistance)
      : 0;
  }

  public double getDynamicFlywheelVelocity(double adjustedDistance) {
    return m_limelight.isTargetVisible()
      ? flywheelVelocityMap.getInterpolated(adjustedDistance)
      : 0;
  }

  public void tunePivotAngle() {
    pivotController.setSetpoint(pivotAngleTunableNumber.get());
  }

  public void tuneFlywheelVelocity() {
    pivotController.setSetpoint(flywheelVelocityTunableNumber.get());
  }

  public void setDynamic() {
    setPivotAngle(getDynamicPivotAngle());
    setFlywheelVelocity(getDynamicFlywheelVelocity());
  }

  public void setMoveAndShoot(double adjustedDistance) {
    setPivotAngle(getDynamicPivotAngle(adjustedDistance));
    setFlywheelVelocity(getDynamicFlywheelVelocity(adjustedDistance));
  }

  public void setCalculatedPivotVoltage() {
    pivotMotor.setVoltage(pivotController.calculate(getPivotAngle()));
  }

  public void setCalculatedFlywheelVoltage() {
    topFlywheelMotor.setVoltage(flywheelController.calculate(getFlywheelVelocity()) + flywheelFeedforward.calculate(getTargetFlywheelVelocity()));
  }

  public void setCalculatedFeederVoltage() {
    feederMotor.setVoltage(feederController.calculate(getFeederVelocity()));
  }

  public Command setPivotAngleCommand(double targetAngle) {
    return new InstantCommand(() -> setPivotAngle(targetAngle));
  }

  public Command setFlywheelVelocityCommand(double targetVelocity) {
    return new InstantCommand(() -> setFlywheelVelocity(targetVelocity));
  }

  public Command setFeederVelocityCommand(double targetVelocity) {
    return new InstantCommand(() -> setFeederVelocity(targetVelocity));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current Pivot Angle", getPivotAngle());
    SmartDashboard.putNumber("Target Pivot Angle", getTargetPivotAngle());
    SmartDashboard.putNumber("Current Flywheel Velocity", getFlywheelVelocity());
    SmartDashboard.putNumber("Target Flywheel Velocity", getTargetFlywheelVelocity());
    SmartDashboard.putBoolean("Dynamic Enabled?", dynamicEnabled);

    if (dynamicEnabled) setDynamic();


    setCalculatedPivotVoltage();
    setCalculatedFlywheelVoltage();
    setCalculatedFeederVoltage();
  }
}
