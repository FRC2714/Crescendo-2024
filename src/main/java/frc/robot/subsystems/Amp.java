// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.AmpConstants.AmpPIDConstants;
import frc.robot.utils.TunableNumber;

public class Amp extends SubsystemBase {
  /** Creates a new Amp. */

  private CANSparkFlex pivotMotor;
  private AbsoluteEncoder pivotEncoder;
  private PIDController pivotController;
  private TunableNumber tunableAngle, tunableP;

  private Mechanism2d ampMech;
  private MechanismRoot2d ampMechRoot;
  private MechanismLigament2d ampMechArm;

  public Amp() {

    pivotMotor = new CANSparkFlex(AmpConstants.kAmpCanId, MotorType.kBrushless);
    pivotMotor.setIdleMode(IdleMode.kBrake);

    pivotMotor.setSmartCurrentLimit(AmpConstants.kSmartCurrentLimit);

    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pivotEncoder.setInverted(true);
    pivotEncoder.setPositionConversionFactor(AmpConstants.kPivotConversionFactor);
    pivotEncoder.setZeroOffset(AmpConstants.kPivotZeroOffset);

    tunableAngle = new TunableNumber("Tuanble Amp Angle");
    tunableP = new TunableNumber("Tunable Amp P");
    tunableAngle.setDefault(0);
    tunableP.setDefault(0);
  
    
    pivotController = new PIDController(AmpPIDConstants.kP, AmpPIDConstants.kI, AmpPIDConstants.kD);
    
    pivotMotor.burnFlash();


    ampMech = new Mechanism2d(3, 3);
    ampMechRoot = ampMech.getRoot("amp", 1, 1);
    ampMechArm = ampMechRoot.append(
      new MechanismLigament2d("ampArm", 1, 45, 6, new Color8Bit(Color.kOrange))
    );
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
    SmartDashboard.putData("Amp Mech2d", ampMech);
    setCalculatedPivotVoltage();

    ampMechArm.setAngle(getPivotAngle());

    // if (tunableAngle.hasChanged()) {
    //   setPivotAngle(tunableAngle.get());
    // }
    // if (tunableP.hasChanged()) {
    //   pivotController.setP(tunableP.get());
    // }
  }
}
