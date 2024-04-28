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
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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

  private double flywheelReference;

  private boolean dynamicEnabled;

  private Mechanism2d shooterMech;
  private MechanismRoot2d shooterMechRoot;
  private MechanismLigament2d shooterMechArm;

  private SingleJointedArmSim shooterSim;
  private EncoderSim encoderSim;

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

    topFlywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 20000);
    bottomFlywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 20000);

    topFlywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20000);
    bottomFlywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20000);

    topFlywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20000);
    bottomFlywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20000);

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

    flywheelReference = 0;

    shooterMech = new Mechanism2d(3, 3);
    shooterMechRoot = shooterMech.getRoot("shooter", 1, 1);
    shooterMechArm = shooterMechRoot.append(
      new MechanismLigament2d("shooterArm", 1, 0, 10, new Color8Bit(Color.kOrange))
    );

    shooterSim = new SingleJointedArmSim(
      DCMotor.getNeoVortex(1),
      ShooterConstants.kPivotGearRatio,
      1,
      1,
      Units.degreesToRadians(ShooterConstants.kMinPivotAngle),
      Units.degreesToRadians(ShooterConstants.kMaxPivotAngle),
      false,
      0);
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

  pivotAngleMap.put(1.64, 37.0);
  pivotAngleMap.put(1.89, 34.0);
  pivotAngleMap.put(2.19, 30.0);
  pivotAngleMap.put(2.39, 27.0);
  pivotAngleMap.put(2.78, 22.0);
  pivotAngleMap.put(3.06, 20.0);  
  pivotAngleMap.put(3.20, 18.0); 
  pivotAngleMap.put(3.51, 15.5);
  pivotAngleMap.put(3.78, 15.0);
  pivotAngleMap.put(4.04, 13.0);
  pivotAngleMap.put(4.31, 11.5);
  pivotAngleMap.put(4.44, 11.0);
  pivotAngleMap.put(4.57, 10.5);
  //  pivotAngleMap.put(1.64, 37.0);
  //  pivotAngleMap.put(1.73, 35.0);
  //  pivotAngleMap.put(1.84, 33.0);
  //  pivotAngleMap.put(1.95, 33.0);
  //  pivotAngleMap.put(2.02, 31.0);
  //  pivotAngleMap.put(2.14, 31.0);
  //  pivotAngleMap.put(2.14, 31.0);
  //  pivotAngleMap.put(2.25, 30.0);
  //  pivotAngleMap.put(2.36, 28.0);
  //  pivotAngleMap.put(2.42, 27.0);
  //  pivotAngleMap.put(2.54, 27.0);
  //  pivotAngleMap.put(2.66, 25.0);
  //  pivotAngleMap.put(2.86, 23.0);
  //  pivotAngleMap.put(2.95, 21.0);
  //  pivotAngleMap.put(3.03, 21.0);
  //  pivotAngleMap.put(3.17, 19.0);
  //  pivotAngleMap.put(3.30, 19.0);
  //  pivotAngleMap.put(3.45, 18.0);
  //  pivotAngleMap.put(3.55, 18.0);
  //  pivotAngleMap.put(3.67, 18.0);
  //  pivotAngleMap.put(3.70, 16.0);
  //  pivotAngleMap.put(3.85, 15.5);
  //  pivotAngleMap.put(3.90, 15.0);
  //  pivotAngleMap.put(4.05, 14.0);
  //  pivotAngleMap.put(4.4, 13.0);
  //  pivotAngleMap.put(4.6, 12.0);
  //  pivotAngleMap.put(4.9, 10.5);
  //  pivotAngleMap.put(5.01, 10.25);
  //  pivotAngleMap.put(5.17, 10.0);
  //  pivotAngleMap.put(5.2, 10.0);
  //  pivotAngleMap.put(5.4, 9.25);
  //  pivotAngleMap.put(5.55, 9.0);
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
    flywheelReference = targetVelocity;
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

      public Command readyShort() {
    return new ParallelCommandGroup(disableDynamic(),
                                    setPivotAngleCommand(ShooterConstants.kPassUnderStageAngle),
                                    setFlywheelVelocityCommand(ShooterConstants.kShortFlywheelVelocity));
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

  public void setCalculatedPivotVoltageSim() {
    shooterSim.setInputVoltage(pivotController.calculate(Units.degreesToRadians(shooterSim.getAngleRads())));
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

  public boolean flywheelAtSetpoint() {
    return Math.abs(flywheelReference - flywheelEncoder.getVelocity()) < 500 && flywheelReference != 0;
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
    SmartDashboard.putData("Shooter Mech2d", shooterMech);

    shooterMechArm.setAngle(getPivotAngle());

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

  @Override
  public void simulationPeriodic() {
    setCalculatedPivotVoltageSim();
    shooterSim.update(0.020);
    SmartDashboard.putNumber("Shooter pivot Sim", shooterSim.getAngleRads());
  }
}