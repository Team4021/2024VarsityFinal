// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForTime extends Command{
  
  private final DriveSubsystem m_robotDrive;
  private final double xSpeed;
  private final double ySpeed;
  private final double rot;

  private double target = 0.0;
  private double counter = 0.0;

  /** Creates a new DriveForTime. */
  public DriveForTime(DriveSubsystem m_robotDrive, double xSpeed, double ySpeed, double rot, double seconds) {
    this.m_robotDrive = m_robotDrive;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rot = rot;

    target = seconds * 50;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
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

    m_robotDrive.drive(xSpeed, ySpeed, rot, true, true);
    SmartDashboard.putNumber("counter", counter);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.drive(0.0,0.0,0.0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter >= target;
  }
}