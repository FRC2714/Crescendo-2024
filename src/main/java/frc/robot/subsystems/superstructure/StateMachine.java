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

public class StateMachine extends SubsystemBase {
  /** Creates a new StateMachine. */
  Superstructure m_superstructure;

  public enum IntakeState {
    INTAKE_BACK,
    INTAKE_FRONT,
    EXTAKE_BACK,
    EXTAKE_FRONT,
    IDLE
  }

  public enum ShooterState {
    DYNAMIC,
    SUBWOOFER,
    PASSTOAMP,
    SOURCETOMID,
    UNDERSTAGE,
    SHORT,
    AMP,
    STOW
  }

  public enum TriggerState {
    RIGHT,
    LEFT,
    NONE
  }

  public enum BumperState {
    RIGHT,
    LEFT,
    NONE
  }

  public enum ClimberState {
    EXTENDED,
    ZERO,
    RETRACTED
  }

  private IntakeState currentIntakeState;
  private TriggerState currentTriggerState;
  private BumperState currentBumperState;
  private ShooterState currentShooterState;
  private ClimberState currentClimberState;
  public StateMachine(Superstructure m_superstructure) {
    this.m_superstructure = m_superstructure;

    currentIntakeState = IntakeState.IDLE;
    currentTriggerState = TriggerState.NONE;
    currentBumperState = BumperState.NONE;
    currentShooterState = ShooterState.STOW;
    currentClimberState = ClimberState.ZERO;
  }

  public void setTriggerState(TriggerState triggerState) {
    currentTriggerState = triggerState;
  }

  public void setBumperState(BumperState bumperState) {
    currentBumperState = bumperState;
  }

  public void setIntakeState(IntakeState intakeState) {
    currentIntakeState = intakeState;
  }

  public void setShooterState(ShooterState shooterState) {
    currentShooterState = shooterState;
  }

  public void setClimberState(ClimberState climberState) {
    currentClimberState = climberState;
  }

  public Command setCurrentIntakeState(IntakeState intakeState) {
    return new InstantCommand(() -> setIntakeState(intakeState));
  }

  public Command setCurrentTriggerState(TriggerState triggerState) {
    return new InstantCommand(() -> setTriggerState(triggerState));
  }

  public Command setCurrentBumperState(BumperState bumperState) {
    return new InstantCommand(() -> setBumperState(bumperState));
  }

  public Command setCurrentShooterState(ShooterState shooterState) {
    return new InstantCommand(() -> setShooterState(shooterState));
  }

  public Command setCurrentClimberState(ClimberState climberState) {
    return new InstantCommand(() -> setClimberState(climberState));
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

  public Command extakeBack() {
    return new SequentialCommandGroup(
      setCurrentBumperState(BumperState.RIGHT),
      m_superstructure.extakeBack()
    );
  }

  public Command extakeFront() {
    return new SequentialCommandGroup(
      setCurrentBumperState(BumperState.LEFT),
      m_superstructure.extakeFront()
    );
  }

  public Command idleIntake() {
    return new SequentialCommandGroup(
      setCurrentTriggerState(TriggerState.NONE),
      m_superstructure.idleIntake()
    );
  }

  public Command idleExtake() {
    return new SequentialCommandGroup(
      setCurrentBumperState(BumperState.NONE),
      m_superstructure.idleExtake()
    );
  }

  public Command stowShooter() {
    return new SequentialCommandGroup(
      setCurrentShooterState(ShooterState.STOW),
      m_superstructure.stowShooter()
    );
  }

  public Command readyShooterToSubwoofer() {
    return new SequentialCommandGroup(
      setCurrentShooterState(ShooterState.SUBWOOFER),
      m_superstructure.readyShooterToSubwoofer()
    );
  }

  public Command readyPassToAmp()
  {
      return new SequentialCommandGroup(
      setCurrentShooterState(ShooterState.PASSTOAMP),
      m_superstructure.readyPassToAmp()
    );
  }

  public Command readyPassToMid()
  {
      return new SequentialCommandGroup(
      setCurrentShooterState(ShooterState.SOURCETOMID),
      m_superstructure.readySourceToMid()
    );
  }

  
  public Command readyPassUnderstage()
  {
      return new SequentialCommandGroup(
      setCurrentShooterState(ShooterState.UNDERSTAGE),
      m_superstructure.readyUnderstage()
    );
  }

  public Command readyShort()
  {
      return new SequentialCommandGroup(
      setCurrentShooterState(ShooterState.SHORT),
      m_superstructure.readyShort()
    );
  }


  public Command readyShooterToAmp() {
    return new SequentialCommandGroup(
      setCurrentShooterState(ShooterState.AMP),
      m_superstructure.readyShooterToAmp()
    );
  }

  public Command enableDynamicShooter() {
    return new SequentialCommandGroup(
      setCurrentShooterState(ShooterState.DYNAMIC),
      m_superstructure.enableDynamicShooter()
    );
  }

  public Command extendClimbers() {
    return new SequentialCommandGroup(
      setCurrentClimberState(ClimberState.EXTENDED),
      m_superstructure.extendClimbers()
    );
  }

  public Command retractClimbers() {
    return new SequentialCommandGroup(
      setCurrentClimberState(ClimberState.RETRACTED),
      m_superstructure.retractClimbers()
    );
  }

  public Command zeroClimbers() {
    return new SequentialCommandGroup(
      setCurrentClimberState(ClimberState.ZERO),
      m_superstructure.zeroClimbers()
    );
  }

  public Command climberSelectCommand(ClimberState climberState) {
    return new SelectCommand<ClimberState>(Map.ofEntries(
      Map.entry(ClimberState.ZERO, zeroClimbers()),
      Map.entry(ClimberState.RETRACTED, retractClimbers()),
      Map.entry(ClimberState.EXTENDED, extendClimbers())
    ), () -> climberState);
  }

  public Command intakeSelectCommand(IntakeState intakeState) {
    // System.out.println(currentTriggerState);
    return new SelectCommand<IntakeState>(Map.ofEntries(
      Map.entry(
        IntakeState.IDLE,
        new SequentialCommandGroup(
          setCurrentIntakeState(intakeState),
          new SelectCommand<IntakeState>(Map.ofEntries(
            Map.entry(IntakeState.INTAKE_BACK, intakeBack()),
            Map.entry(IntakeState.INTAKE_FRONT, intakeFront()),
            Map.entry(IntakeState.EXTAKE_BACK, extakeBack()),
            Map.entry(IntakeState.EXTAKE_FRONT, extakeFront()),
            Map.entry(IntakeState.IDLE, idleIntake())
          ), () -> currentIntakeState))
      ),
      Map.entry(
        IntakeState.INTAKE_BACK,
        currentTriggerState == TriggerState.NONE ?
        new SequentialCommandGroup(
          setCurrentIntakeState(IntakeState.IDLE),
          idleIntake())
        : new InstantCommand()
      ),
      Map.entry(
        IntakeState.INTAKE_FRONT,
        currentTriggerState == TriggerState.NONE ?
        new SequentialCommandGroup(
          setCurrentIntakeState(IntakeState.IDLE),
          idleIntake())
        : new InstantCommand()
      ),
      Map.entry(
        IntakeState.EXTAKE_BACK,
        currentBumperState == BumperState.NONE ?
        new SequentialCommandGroup(
          setCurrentIntakeState(IntakeState.IDLE),
          idleExtake())
        : new InstantCommand()
      ),
      Map.entry(
        IntakeState.EXTAKE_FRONT,
        currentBumperState == BumperState.NONE ?
        new SequentialCommandGroup(
          setCurrentIntakeState(IntakeState.IDLE),
          idleExtake())
        : new InstantCommand()
      )),
    () -> currentIntakeState);
  }

  public Command shooterSelectCommand(ShooterState shooterState) {
    return new SelectCommand<ShooterState>(Map.ofEntries(
      Map.entry(ShooterState.STOW, stowShooter()),
      Map.entry(ShooterState.SUBWOOFER, readyShooterToSubwoofer()),
      Map.entry(ShooterState.AMP, readyShooterToAmp()),
      Map.entry(ShooterState.DYNAMIC, enableDynamicShooter()),
      Map.entry(ShooterState.PASSTOAMP, readyPassToAmp()),
      Map.entry(ShooterState.SOURCETOMID, readyPassToMid()),
      Map.entry(ShooterState.UNDERSTAGE, readyPassUnderstage()),
      Map.entry(ShooterState.SHORT, readyShort())
    ), () -> shooterState);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Intake State", currentIntakeState.toString());
    SmartDashboard.putString("Trigger State", currentTriggerState.toString());
    SmartDashboard.putString("Bumper State", currentBumperState.toString());
    SmartDashboard.putString("Shooter State", currentShooterState.toString());
    SmartDashboard.putString("Climber State", currentClimberState.toString());
  }
}
