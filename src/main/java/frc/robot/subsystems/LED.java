// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LED extends SubsystemBase {
  private Spark m_blinkin;

  /** Creates a new LED. */
  public LED() {
    m_blinkin = new Spark(LEDConstants.kBlinkinPort);
  }

  public void set(double val) {
    m_blinkin.set(val);
  }

  public Command setFire() {
    return new InstantCommand(() -> set(LEDConstants.kFire));
  }

  public Command setGreen() {
    return new InstantCommand(() -> set(LEDConstants.kGreen));
  }

  public Command setRed() {
    return new InstantCommand(() -> set(LEDConstants.kRed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}