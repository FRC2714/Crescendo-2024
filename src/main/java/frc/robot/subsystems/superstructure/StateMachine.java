// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class StateMachine extends SubsystemBase {
  /** Creates a new StateMachine. */
  Superstructure m_superstructure;

  public enum IntakeState {
    BACK,
    FRONT,
    IDLE
  }

  IntakeState currentIntakeState;

  public StateMachine(DriveSubsystem m_drivetrain) {
    m_superstructure = new Superstructure(m_drivetrain);

    currentIntakeState = IntakeState.IDLE;
  }

  public Command setCurrentIntakeState(IntakeState intakeState) {
    return new InstantCommand(() -> currentIntakeState = intakeState);
  }

  public Command intakeSelectCommand(IntakeState intakeState) {
    return new SelectCommand<IntakeState>(Map.ofEntries(
      Map.entry(
        IntakeState.IDLE,
        new SequentialCommandGroup(
          setCurrentIntakeState(intakeState),
          new SelectCommand<IntakeState>(Map.ofEntries(
            Map.entry(IntakeState.BACK, m_superstructure.intakeBack()),
            Map.entry(IntakeState.FRONT, m_superstructure.intakeFront()),
            Map.entry(IntakeState.IDLE, m_superstructure.idle())
          ), () -> intakeState))
      ),
      Map.entry(
        IntakeState.BACK,
        new SequentialCommandGroup(
          setCurrentIntakeState(currentIntakeState == IntakeState.FRONT ? IntakeState.BACK : intakeState),
          new SelectCommand<IntakeState>(Map.ofEntries(
            Map.entry(IntakeState.BACK, m_superstructure.intakeBack()),
            Map.entry(IntakeState.FRONT, m_superstructure.intakeFront()),
            Map.entry(IntakeState.IDLE, m_superstructure.idle())
          ), () -> intakeState))
      ),
      Map.entry(
        IntakeState.FRONT,
        new SequentialCommandGroup(
          setCurrentIntakeState(currentIntakeState == IntakeState.BACK ? IntakeState.FRONT : intakeState),
          new SelectCommand<IntakeState>(Map.ofEntries(
            Map.entry(IntakeState.BACK, m_superstructure.intakeBack()),
            Map.entry(IntakeState.FRONT, m_superstructure.intakeFront()),
            Map.entry(IntakeState.IDLE, m_superstructure.idle())
          ), () -> intakeState))
      )
    ), () -> currentIntakeState);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Intake State", currentIntakeState.toString());
  }
}
