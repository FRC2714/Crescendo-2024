// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

    public enum IntakeState {
      FRONT,
      BACK
    }

    private Intake m_intake;
    private IntakeState m_state;

  /** Creates a new IntakeBackCommand. */
  public IntakeCommand(Intake m_intake, IntakeState m_state) {
    this.m_intake = m_intake;
    this.m_state = m_state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_state) {
      case BACK:
        m_intake.setBackBottomRollerVoltage(-IntakeConstants.kBackBottomRollerVoltageBackSide);
        m_intake.setBackDirectionRollerVoltage(IntakeConstants.kBackDirectionRollerVoltageBackSide);
        m_intake.setFeederVoltage(IntakeConstants.kFeederIntakeVoltage);
        break;
      case FRONT:
        m_intake.setFrontRollerVoltage(IntakeConstants.kFrontRollerVoltage);
        m_intake.setBackBottomRollerVoltage(IntakeConstants.kBackBottomRollerVoltageFrontSide);
        m_intake.setBackDirectionRollerVoltage(IntakeConstants.kBackDirectionRollerVoltageFrontSide);
        m_intake.setConveyorVoltage(-IntakeConstants.kConveyorVoltage);
        m_intake.setFeederVoltage(IntakeConstants.kFeederIntakeVoltage);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    switch (m_state) {
      case BACK:
        m_intake.setBackBottomRollerVoltage(0);
        m_intake.setBackDirectionRollerVoltage(0);
        m_intake.setFeederVoltage(0);
        break;
      case FRONT:
        m_intake.setFrontRollerVoltage(0);
        m_intake.setBackBottomRollerVoltage(0);
        m_intake.setBackDirectionRollerVoltage(0);
        m_intake.setConveyorVoltage(0);
        m_intake.setFeederVoltage(0);
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
