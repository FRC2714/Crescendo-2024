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

  public enum TriggerState {
    RIGHT,
    LEFT,
    NONE
  }

  private IntakeState currentIntakeState;
  private TriggerState currentTriggerState;

  public StateMachine(DriveSubsystem m_drivetrain) {
    m_superstructure = new Superstructure(m_drivetrain);

    currentIntakeState = IntakeState.IDLE;
    currentTriggerState = TriggerState.NONE;
  }

  public TriggerState getCurrentTriggerState() {
    return currentTriggerState;
  }

  public void setTriggerState(TriggerState triggerState) {
    currentTriggerState = triggerState;
  }

  public void setIntakeState(IntakeState intakeState) {
    currentIntakeState = intakeState;
  }

  public Command setCurrentIntakeState(IntakeState intakeState) {
    return new InstantCommand(() -> setIntakeState(intakeState));
  }

  public Command setCurrentTriggerState(TriggerState triggerState) {
    return new InstantCommand(() -> setTriggerState(triggerState));
  }

  public Command intakeBack() {
    return new SequentialCommandGroup(
      setCurrentTriggerState(TriggerState.RIGHT),
      m_superstructure.intakeBack()
    );
  }

  public Command intakeFront() {
    return new SequentialCommandGroup(
      setCurrentTriggerState(TriggerState.LEFT),
      m_superstructure.intakeFront()
    );
  }

  public Command idle() {
    return new SequentialCommandGroup(
      setCurrentTriggerState(TriggerState.NONE),
      m_superstructure.idle()
    );
  }

  public Command intakeSelectCommand(IntakeState intakeState) {
    System.out.println(currentTriggerState);
    return new SelectCommand<IntakeState>(Map.ofEntries(
      Map.entry(
        IntakeState.IDLE,
        new SequentialCommandGroup(
          setCurrentIntakeState(intakeState),
          new SelectCommand<IntakeState>(Map.ofEntries(
            Map.entry(IntakeState.BACK, intakeBack()),
            Map.entry(IntakeState.FRONT, intakeFront()),
            Map.entry(IntakeState.IDLE, idle())
          ), () -> currentIntakeState))
      ),
      Map.entry(
        IntakeState.BACK,
        currentTriggerState == TriggerState.NONE ?
        new SequentialCommandGroup(
          setCurrentIntakeState(IntakeState.IDLE),
          idle())
        : new InstantCommand()
      ),
      Map.entry(
        IntakeState.FRONT,
        currentTriggerState == TriggerState.NONE ?
        new SequentialCommandGroup(
          setCurrentIntakeState(IntakeState.IDLE),
          idle())
        : new InstantCommand()
      )),
    () -> currentIntakeState);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Intake State", currentIntakeState.toString());
    SmartDashboard.putString("Trigger State", currentTriggerState.toString());
  }
}
