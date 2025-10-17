// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.AmpConfig;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.AmpConstants.AmpPIDConstants;
import frc.robot.utils.TunableNumber;;


public class Amp extends SubsystemBase {
  /** Creates a new Amp. */

  private SparkFlex pivotMotor;
  private AbsoluteEncoder pivotEncoder;
  private PIDController pivotController;
  private TunableNumber tunableAngle, tunableP;

  public Amp() {

    pivotMotor = new SparkFlex(AmpConstants.kAmpCanId, MotorType.kBrushless);

    tunableAngle = new TunableNumber("Tuanble Amp Angle");
    tunableP = new TunableNumber("Tunable Amp P");
    tunableAngle.setDefault(0);
    tunableP.setDefault(0);
  
    
    pivotController = new PIDController(AmpPIDConstants.kP, AmpPIDConstants.kI, AmpPIDConstants.kD);
    
    pivotMotor.configure(AmpConfig.pivot, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getPivotAngle() {
    return ((pivotEncoder.getPosition() - AmpConstants.kPivotEncoderKinematicOffset) / AmpConstants.kPivotGearRatio);
  }

  public double getTargetPivotAngle() {
    return pivotController.getSetpoint();
  }

  public void setPivotAngle(double targetAngle) {
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
    SmartDashboard.putNumber("Amp goal", pivotController.getSetpoint());
    SmartDashboard.putNumber("Amp P goal", pivotController.getP());
    setCalculatedPivotVoltage();

    // if (tunableAngle.hasChanged()) {
    //   setPivotAngle(tunableAngle.get());
    // }
    // if (tunableP.hasChanged()) {
    //   pivotController.setP(tunableP.get());
    // }
  }
}
