// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntermediateSubsystem;

public class IntakeForTime extends Command {
  private final IntakeSubsystem m_intake;
  private final IntermediateSubsystem m_inter;
  private double target = 0.0;
  private double counter = 0.0;
  private DigitalInput m_limitSwitch;

  /** Creates a new IntakeForTime. */
  public IntakeForTime(IntakeSubsystem m_intake, IntermediateSubsystem m_inter, double seconds, DigitalInput m_limitSwitch) {
    this.m_intake = m_intake;
    this.m_inter = m_inter;
    this.m_limitSwitch = m_limitSwitch;

    target = seconds * 50;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_inter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(counter < target){
      counter++;
    }

    if (RobotContainer.m_limitSwitch.get() == false){
        m_intake.runIntake(0.3);
        m_inter.runIntermediate(0.15);
    } else if (RobotContainer.m_limitSwitch.get() == true){
        m_intake.noRunIntake();
        m_inter.noRunIntermediate();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.noRunIntake();
    m_inter.noRunIntermediate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter >= target;  
}
}