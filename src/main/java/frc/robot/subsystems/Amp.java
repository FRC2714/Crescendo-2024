// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;

public class Amp extends SubsystemBase {
  /** Creates a new Amp. */

  private Servo leftPivotServo;
  private Servo rightPivotServo;

  public Amp() {

    leftPivotServo = new Servo(AmpConstants.kLeftAmpPivotChannel);
    rightPivotServo = new Servo(AmpConstants.kRightAmpPivotChannel);

  }

    public double getTargetPosition() {
      return leftPivotServo.get();
    }

    public void setPivot(double target) {
      leftPivotServo.setPosition(1 - target);
      rightPivotServo.setPosition(target);
    }
    public Command stow(){
      return new InstantCommand(() -> setPivot(0)); // tbd 
    }

    public Command extend(){
      return new InstantCommand(() -> setPivot(0.65)); //tbd
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
