// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntermediateSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootForTime extends Command {
  private final IntakeSubsystem m_intake;
  private final IntermediateSubsystem m_inter;
  private final ShooterSubsystem m_shoot;
  private double target = 0.0;
  private double counter = 0.0;
  /** Creates a new IntakeForTime. */
  public ShootForTime(ShooterSubsystem m_shoot, IntakeSubsystem m_intake, IntermediateSubsystem m_inter, double seconds) {
    this.m_shoot = m_shoot;
    this.m_intake = m_intake;
    this.m_inter = m_inter;


    target = seconds * 50;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shoot, m_intake, m_inter);
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
    m_shoot.shoot(0.7);
    if(counter > 30){
        m_inter.runIntermediate(0.7);
        m_intake.runIntake(0.3);
    }
  

    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.noRunIntake();
    m_inter.noRunIntermediate();
    m_shoot.noRunshoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter >= target;  
    }
}
