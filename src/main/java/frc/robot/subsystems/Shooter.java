// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.FlywheelPIDConstants;
import frc.robot.Constants.ShooterConstants.PivotPIDConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.InterpolatingTreeMap;
import frc.robot.utils.TunableNumber;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private CANSparkFlex topFlywheelMotor;
  private CANSparkFlex bottomFlywheelMotor;
  private CANSparkFlex pivotMotor;

  private AbsoluteEncoder pivotEncoder;
  private RelativeEncoder flywheelEncoder;

  private PIDController pivotController;
  private SparkPIDController flywheelController;

  private Vision m_vision;

  private InterpolatingTreeMap pivotAngleMap;
  private InterpolatingTreeMap flywheelVelocityMap;
  private InterpolatingTreeMap shootTimeMap;

  private TunableNumber pivotAngleTunableNumber;
  private TunableNumber flywheelVelocityTunableNumber;
  private TunableNumber pivotP, flywheelP, flywheelV, flywheelD;

  private boolean dynamicEnabled;


  public Shooter(Vision m_vision) {
    pivotMotor = new CANSparkFlex(ShooterConstants.kPivotCanId, MotorType.kBrushless);
    topFlywheelMotor = new CANSparkFlex(ShooterConstants.kTopFlywheelCanId, MotorType.kBrushless);
    bottomFlywheelMotor = new CANSparkFlex(ShooterConstants.kBottomFlywheelCanId, MotorType.kBrushless);

    pivotMotor.setIdleMode(IdleMode.kBrake);
    topFlywheelMotor.setIdleMode(IdleMode.kCoast);
    bottomFlywheelMotor.setIdleMode(IdleMode.kCoast);

    topFlywheelMotor.setInverted(true);

    pivotMotor.setSmartCurrentLimit(ShooterConstants.kPivotSmartCurrentLimit);
    topFlywheelMotor.setSmartCurrentLimit(ShooterConstants.kTopFlywheelSmartCurrentLimit);
    bottomFlywheelMotor.setSmartCurrentLimit(ShooterConstants.kBottomFlywheelSmartCurrentLimit);

    bottomFlywheelMotor.follow(topFlywheelMotor, false);

    pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    pivotEncoder.setPositionConversionFactor(ShooterConstants.kPivotEncoderConversionFactor);
    pivotEncoder.setInverted(true);
    pivotEncoder.setZeroOffset(ShooterConstants.kPivotEncoderZeroOffset);

    pivotMotor.setInverted(true);

    flywheelEncoder = topFlywheelMotor.getEncoder();

    flywheelEncoder.setVelocityConversionFactor(ShooterConstants.kFlywheelGearRatio);

    topFlywheelMotor.enableVoltageCompensation(ShooterConstants.kNominalVoltage);
    bottomFlywheelMotor.enableVoltageCompensation(ShooterConstants.kNominalVoltage);
    pivotMotor.enableVoltageCompensation(ShooterConstants.kNominalVoltage);

    pivotController = new PIDController(PivotPIDConstants.kP, PivotPIDConstants.kI, PivotPIDConstants.kD);
    flywheelController = topFlywheelMotor.getPIDController();

    flywheelController.setP(FlywheelPIDConstants.kP);
    flywheelController.setFF(FlywheelPIDConstants.kFF);
    topFlywheelMotor.burnFlash();
    bottomFlywheelMotor.burnFlash();
    pivotMotor.burnFlash();

    pivotAngleMap = new InterpolatingTreeMap();
    flywheelVelocityMap = new InterpolatingTreeMap();
    shootTimeMap = new InterpolatingTreeMap();

    dynamicEnabled = false;
    
    populatePivotAngleMap();
    populateFlywheelVelocityMap();
    populateShootTimeMap();

    pivotAngleTunableNumber = new TunableNumber("Tunable Pivot Angle");
    flywheelVelocityTunableNumber = new TunableNumber("Tunable Flywheel Velocity");
    pivotP = new TunableNumber("Pivot P");
    flywheelP = new TunableNumber("Flywheel P");
    flywheelV = new TunableNumber("Flywheel FF");
    flywheelD = new TunableNumber("Flywheel D");


    pivotAngleTunableNumber.setDefault(0);
    flywheelVelocityTunableNumber.setDefault(0);
    flywheelD.setDefault(0);
    pivotP.setDefault(0);
    flywheelP.setDefault(0);
    flywheelV.setDefault(0);
    this.m_vision = m_vision;
  }

  public void toggleDynamic() {
    dynamicEnabled = !dynamicEnabled;
  }

  public Command enableDynamic() {
    return new InstantCommand(() -> dynamicEnabled = true);
  }

  public Command disableDynamic() {
    return new InstantCommand(() -> dynamicEnabled = false);
  }

  public void populatePivotAngleMap() {
    pivotAngleMap.put(1.61, 35.0);
   pivotAngleMap.put(1.73, 35.0);
   pivotAngleMap.put(1.84, 33.0);
   pivotAngleMap.put(1.95, 33.0);
   pivotAngleMap.put(2.02, 31.0);
   pivotAngleMap.put(2.14, 31.0);
   pivotAngleMap.put(2.14, 31.0);
   pivotAngleMap.put(2.25, 30.0);
   pivotAngleMap.put(2.36, 28.0);
   pivotAngleMap.put(2.42, 27.0);
   pivotAngleMap.put(2.54, 27.0);
   pivotAngleMap.put(2.66, 25.0);
   pivotAngleMap.put(2.86, 23.0);
   pivotAngleMap.put(2.95, 21.0);
   pivotAngleMap.put(3.03, 21.0);
   pivotAngleMap.put(3.17, 19.0);
   pivotAngleMap.put(3.30, 19.0);
   pivotAngleMap.put(3.45, 18.0);
   pivotAngleMap.put(3.55, 18.0);
   pivotAngleMap.put(3.67, 18.0);
   pivotAngleMap.put(3.70, 16.0);
   pivotAngleMap.put(3.85, 15.5);
   pivotAngleMap.put(3.90, 15.0);
   pivotAngleMap.put(4.05, 14.0);
   pivotAngleMap.put(4.4, 13.0);
   pivotAngleMap.put(4.6, 12.0);
   pivotAngleMap.put(4.9, 11.5);
   pivotAngleMap.put(5.01, 11.25);
   pivotAngleMap.put(5.17, 11.0);
   pivotAngleMap.put(5.2, 11.0);
   pivotAngleMap.put(5.4, 10.25);
   pivotAngleMap.put(5.55, 10.0);














  //  pivotAngleMap.put(3.91, 16.0);
  //  pivotAngleMap.put(4.05, 15.0);
  //  pivotAngleMap.put(4.13, 15.0);
  //  pivotAngleMap.put(4.28, 15.0);
  //  pivotAngleMap.put(4.38, 14.0);
  //  pivotAngleMap.put(4.47, 14.0);
  //  pivotAngleMap.put(4.59, 14.0);
  //  pivotAngleMap.put(4.65, 13.5);
  //  pivotAngleMap.put(4.74, 13.5);
  //  pivotAngleMap.put(4.84, 13.0);
  //  pivotAngleMap.put(4.96, 13.0);
  //  pivotAngleMap.put(5.06, 12.0);
  //  pivotAngleMap.put(5.16, 12.0);
  //  pivotAngleMap.put(5.27, 12.0);
  //  pivotAngleMap.put(5.34, 11.75);
  //  pivotAngleMap.put(5.47, 11.50);

    //pivotAngleMap.put(null, null);
    //------------------------
    // pivotAngleMap.put(1.82, 35.0);
    // pivotAngleMap.put(2.1, 32.0);
    // pivotAngleMap.put(2.6, 27.0);
    // pivotAngleMap.put(2.80, 25.0);
    // pivotAngleMap.put(3.0, 23.0);
    // pivotAngleMap.put(3.25, 21.0);
    // pivotAngleMap.put(3.40, 19.0);
    // pivotAngleMap.put(3.55, 18.0);
    // pivotAngleMap.put(3.9, 16.0);
    // pivotAngleMap.put(4.0, 15.0);
    // pivotAngleMap.put(4.3, 14.0);
    // pivotAngleMap.put(4.6, 13.0);
    // pivotAngleMap.put(4.9, 12.0);
    // pivotAngleMap.put(5.1, 11.5);
    // pivotAngleMap.put(5.4, 10.5);
    // pivotAngleMap.put(5.8, 10.0);
    // pivotAngleMap.put(6.0, 9.5);
    //-------------------------------
    // pivotAngleMap.put(1.82, 35.0);
    // pivotAngleMap.put(2.1, 32.0);
    // pivotAngleMap.put(2.6, 27.0);
    // pivotAngleMap.put(2.80, 25.0);
    // pivotAngleMap.put(3.0, 23.0);
    // pivotAngleMap.put(3.25, 21.0);
    // pivotAngleMap.put(3.40, 19.0);
    // pivotAngleMap.put(3.55, 18.0);
    // pivotAngleMap.put(3.9, 16.0);
    // pivotAngleMap.put(4.0, 15.0);
    // pivotAngleMap.put(4.3, 14.0);
    // pivotAngleMap.put(4.6, 13.0);
    // pivotAngleMap.put(4.9, 12.0);
    // pivotAngleMap.put(5.1, 11.5);
    // pivotAngleMap.put(5.4, 10.5);
    // pivotAngleMap.put(5.8, 10.0);
    // pivotAngleMap.put(6.0, 9.5);
    //-------------------------------
    // pivotAngleMap.put(1.62, 35.0);
    // pivotAngleMap.put(1.82, 33.0);
    // pivotAngleMap.put(2.16, 28.0);
    // pivotAngleMap.put(2.42, 25.0);
    // pivotAngleMap.put(2.86, 21.0);
    // pivotAngleMap.put(3.11, 19.0);
    // pivotAngleMap.put(3.40, 18.0);
    // pivotAngleMap.put(3.64, 16.0);
    // pivotAngleMap.put(4.0, 15.0);
    // pivotAngleMap.put(4.3, 14.0);
    // pivotAngleMap.put(4.52, 13.0);
    // pivotAngleMap.put(4.82, 12.5);
    //---------------------------
    // pivotAngleMap.put(0.93, 35.0);
    // pivotAngleMap.put(1.7, 27.0);
    // pivotAngleMap.put(2.23, 22.0);
    // pivotAngleMap.put(2.74, 18.0);
    // pivotAngleMap.put(2.74, 18.0);
    // pivotAngleMap.put(3.43, 15.0);
    // pivotAngleMap.put(3.43, 15.0);
    // pivotAngleMap.put(3.87, 13.0);
    // pivotAngleMap.put(4.31, 11.5);
    // pivotAngleMap.put(5.02, 10.0);
    // pivotAngleMap.put(5.77, 8.0);
    // pivotAngleMap.put(5.02, 10.0);
    // pivotAngleMap.put(6.40, 7.0);
    // pivotAngleMap.put(6.40, 7.0);
    //------------------------------
    // pivotAngleMap.put(0.63, 45.0);
    // pivotAngleMap.put(0.88966183597416184, 42.0);
    // pivotAngleMap.put(1.542, 30.0);
    // pivotAngleMap.put(2.35, 20.0);
    // pivotAngleMap.put(2.017, 25.0);
    // pivotAngleMap.put(2.94, 15.0);
    // pivotAngleMap.put(3.76, 14.0);
    // pivotAngleMap.put(4.363, 12.0);
    // pivotAngleMap.put(5.10, 11.7);
    // pivotAngleMap.put(5.49, 10.7);

  }

  public void populateFlywheelVelocityMap() {
    flywheelVelocityMap.put(0.74, 8000.0);
    flywheelVelocityMap.put(0.889661835916184, 8000.0);
    flywheelVelocityMap.put(2.082, 8000.0);
    flywheelVelocityMap.put(2.35, 8000.0);
    flywheelVelocityMap.put(3.13, 8000.0);
    flywheelVelocityMap.put(4.436, 8000.0);
    flywheelVelocityMap.put(5.662, 8000.0);
  }

  public void populateShootTimeMap() {
    shootTimeMap.put(0.74, 0.48);
    shootTimeMap.put(0.889661835916184, 0.22);
    shootTimeMap.put(1.542, 0.36);
    shootTimeMap.put(2.017, 0.32);
    shootTimeMap.put(2.35, 0.51);
    shootTimeMap.put(2.94, 0.44);
    shootTimeMap.put(3.76, 0.45);
    shootTimeMap.put(4.363, 0.57);
    shootTimeMap.put(5.10, 0.59);
    shootTimeMap.put(5.49, 0.95);
  }

  public double getInterpolatedShootTime(double distance) {
    return shootTimeMap.getInterpolated(distance);
  }

  public double getPivotAngle() {
    return ((pivotEncoder.getPosition() - ShooterConstants.kPivotEncoderKinematicOffset) / ShooterConstants.kPivotGearRatio) ;
  }

  public double getTargetPivotAngle() {
    return pivotController.getSetpoint();
  }

  public void setPivotAngle(double targetAngle) {
    if (targetAngle > ShooterConstants.kMaxPivotAngle)
      pivotController.setSetpoint(ShooterConstants.kMaxPivotAngle);
    else if (targetAngle < ShooterConstants.kMinPivotAngle)
      pivotController.setSetpoint(ShooterConstants.kMinPivotAngle);
    else
      pivotController.setSetpoint(targetAngle);
  }

  public double getFlywheelVelocity() {
    return flywheelEncoder.getVelocity();
  }

  public void setFlywheelVelocity(double targetVelocity) {
    flywheelController.setReference(targetVelocity, ControlType.kVelocity);
  }

  public double getDynamicPivotAngle() {
    return pivotAngleMap.getInterpolated(m_vision.getDistanceToGoalMeters());
  }

  public double getDynamicFlywheelVelocity() {
    return flywheelVelocityMap.getInterpolated(m_vision.getDistanceToGoalMeters());
  }

  public double getDynamicPivotAngle(double adjustedDistance) {
    return pivotAngleMap.getInterpolated(adjustedDistance);
  }

  public double getDynamicFlywheelVelocity(double adjustedDistance) {
    return flywheelVelocityMap.getInterpolated(adjustedDistance);
  }

  public Command readyAmp() {
    return new ParallelCommandGroup(disableDynamic(),
                                    setPivotAngleCommand(ShooterConstants.kAmpAngle),
                                    setFlywheelVelocityCommand(ShooterConstants.kAmpFlywheelVelocity));
  }

  public Command readyAmpTest() {
    return new ParallelCommandGroup(disableDynamic(),
                                    setPivotAngleCommand(ShooterConstants.kAmpAngle),
                                    setFlywheelVelocityCommand(ShooterConstants.kAmpFlywheelVelocity));
  }

  public Command readySubwoofer() {
    return new ParallelCommandGroup(disableDynamic(),
                                    setPivotAngleCommand(ShooterConstants.kSubwooferAngle),
                                    setFlywheelVelocityCommand(ShooterConstants.kSubwooferFlywheelVelocity));
  }

  public Command readyPassToAmp() {
    return new ParallelCommandGroup(disableDynamic(),
                                    setPivotAngleCommand(ShooterConstants.kPassToAmpAngle),
                                    setFlywheelVelocityCommand(ShooterConstants.kPassToAmpFlywheelVelocity));
  }

    public Command readySourceToMid() {
    return new ParallelCommandGroup(disableDynamic(),
                                    setPivotAngleCommand(ShooterConstants.kSourceToMidAngle),
                                    setFlywheelVelocityCommand(ShooterConstants.kPassToMidFlywheelVelocity));
  }

      public Command readyUnderstage() {
    return new ParallelCommandGroup(disableDynamic(),
                                    setPivotAngleCommand(ShooterConstants.kPassUnderStageAngle),
                                    setFlywheelVelocityCommand(ShooterConstants.kPassUnderStageFlywheelVelocity));
  }





  public Command readyAllianceZone() {
    return new ParallelCommandGroup(disableDynamic(),
                                    setPivotAngleCommand(30),
                                    setFlywheelVelocityCommand(8000));
  }


  public Command incrementPivotAngle() {
    return new ParallelCommandGroup(disableDynamic(),
                                    setPivotAngleCommand(pivotController.getSetpoint() + 1),
                                    setFlywheelVelocityCommand(8000));
  }

  public Command decrementPivotAngle() {
    return new ParallelCommandGroup(disableDynamic(),
                                    setPivotAngleCommand(pivotController.getSetpoint() - 1),
                                    setFlywheelVelocityCommand(8000));
  }

  public Command readyAmpSeparate() {
    return new ParallelCommandGroup(disableDynamic(),
                                    setPivotAngleCommand(ShooterConstants.kAmpAngle),
                                    setFlywheelVelocityCommand(ShooterConstants.kAmpFlywheelVelocity));
  }

  public void tunePivotAngle() {
    setPivotAngle(pivotAngleTunableNumber.get());
  }

  public void tuneFlywheelVelocity() {
    setFlywheelVelocity(flywheelVelocityTunableNumber.get());
  }

  public void tunePivotP() {
    pivotController.setP(pivotP.get());
  }

  public void tuneFlywheelP() {
    flywheelController.setP(flywheelP.get());
  }

  public void tuneFlywheelD() {
    flywheelController.setD(flywheelD.get());
  }

  public void tuneFlywheelV() {
    flywheelController.setFF(flywheelV.get());
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

  public ParallelCommandGroup stow() {
    return new ParallelCommandGroup(
      disableDynamic(),
      setPivotAngleCommand(0),
      setFlywheelVelocityCommand(0)
    );
  }
  
  public void setShoot() {
    setPivotAngle(35);
    setFlywheelVelocity(1000);
    try {
      TimeUnit.SECONDS.sleep(1);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    
  }

  public ParallelCommandGroup setupShot(double shootingDistance) {
    return new ParallelCommandGroup(
    new InstantCommand(() -> setPivotAngle(shootingDistance)),
    new InstantCommand(() -> setFlywheelVelocity(8000))
    );
  }

  public Command setPivotAngleCommand(double targetAngle) {
    return new InstantCommand(() -> setPivotAngle(targetAngle));
  }

  public Command setFlywheelVelocityCommand(double targetVelocity) {
    return new InstantCommand(() -> setFlywheelVelocity(targetVelocity));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Current Pivot Angle", getPivotAngle());
    SmartDashboard.putNumber("Target Pivot Angle", getTargetPivotAngle());
    SmartDashboard.putNumber("Current Flywheel Velocity", getFlywheelVelocity());
    SmartDashboard.putBoolean("Dynamic Enabled?", dynamicEnabled);
    SmartDashboard.putNumber("Pivot Current", pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Flywheel Current", topFlywheelMotor.getOutputCurrent());

    if (pivotAngleTunableNumber.hasChanged()) {
      tunePivotAngle();
    }

    if (pivotP.hasChanged()) {
      tunePivotP();
    }

    if (flywheelVelocityTunableNumber.hasChanged()) {
      tuneFlywheelVelocity();
    }

    if (flywheelP.hasChanged()) {
      tuneFlywheelP();
    }

    if (flywheelV.hasChanged()) {
      tuneFlywheelV();
    }

    if (flywheelD.hasChanged()) {
      tuneFlywheelD();
    }

    if (dynamicEnabled) setDynamic();

    setCalculatedPivotVoltage();
  }
}