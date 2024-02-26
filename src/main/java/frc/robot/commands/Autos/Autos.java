// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveForTime;
import frc.robot.commands.IntakeForTime;
import frc.robot.commands.ShootForTime;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntermediateSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(DriveSubsystem subsystem) {

    return Commands.sequence();
  }
  public static Command straightAuto(DriveSubsystem m_drive, IntakeSubsystem m_intake, IntermediateSubsystem m_inter, ShooterSubsystem m_shoot, DigitalInput m_limitSwitch){
    return Commands.sequence(    
    new ShootForTime(m_shoot, m_intake, m_inter, 2),
    new DriveForTime(m_drive, 0.2, 0, 0, 0.5),
    new IntakeForTime(m_intake, m_inter, 2, m_limitSwitch),
    new DriveForTime(m_drive, 0, 0, 0, 0),
    new DriveForTime(m_drive, -0.2, 0, 0, 2.5),
    new DriveForTime(m_drive, 0, 0, 0, 0),
    new ShootForTime(m_shoot, m_intake, m_inter, 2)
    );
  }
  public static Command turnAutoBlue(DriveSubsystem m_drive, IntakeSubsystem m_intake, IntermediateSubsystem m_inter, ShooterSubsystem m_shoot, DigitalInput m_limitSwitch){
    return Commands.sequence(
    new ShootForTime(m_shoot, m_intake, m_inter, 1.5),
    new DriveForTime(m_drive, 0.4, 0, 0, 0.5),
    new IntakeForTime(m_intake, m_inter, 1, m_limitSwitch),
    new DriveForTime(m_drive, 0, 0, 0, 0),
    new DriveForTime(m_drive, -0.4, 0, 0, 1),
    new DriveForTime(m_drive, 0, 0, 0, 0),
    new ShootForTime(m_shoot, m_intake, m_inter, 1.5),
    new DriveForTime(m_drive, 0.3, 0, -0.26, 1),
    new DriveForTime(m_drive, 0, -0.26, 0, 1),
    new IntakeForTime(m_intake, m_inter, 1, m_limitSwitch),
    new DriveForTime(m_drive, 0, 0.43, 0, 1.2),
    new DriveForTime(m_drive, -0.35, 0, 0.24, 1),
    new ShootForTime(m_shoot, m_intake, m_inter, 2)
    );
  }
  public static Command nothing(DriveSubsystem m_drive){
    return Commands.sequence(
      new DriveForTime(m_drive, 0.3, 0, 0, 1)
    );
  }
    public static Command turnAutoRed(DriveSubsystem m_drive, IntakeSubsystem m_intake, IntermediateSubsystem m_inter, ShooterSubsystem m_shoot, DigitalInput m_limitSwitch){
    return Commands.sequence(
    new ShootForTime(m_shoot, m_intake, m_inter, 1.5),
    new DriveForTime(m_drive, 0.4, 0, 0, 0.5),
    new IntakeForTime(m_intake, m_inter, 1, m_limitSwitch),
    new DriveForTime(m_drive, 0, 0, 0, 0),
    new DriveForTime(m_drive, -0.4, 0, 0, 1),
    new DriveForTime(m_drive, 0, 0, 0, 0),
    new ShootForTime(m_shoot, m_intake, m_inter, 1.5),
    new DriveForTime(m_drive, 0.3, 0, 0.26, 1),
    new DriveForTime(m_drive, 0, 0.26, 0, 1),
    new IntakeForTime(m_intake, m_inter, 1, m_limitSwitch),
    new DriveForTime(m_drive, 0, -0.43, 0, 1.2),
    new DriveForTime(m_drive, -0.35, 0, -0.24, 1),
    new ShootForTime(m_shoot, m_intake, m_inter, 2)
    );
    }

  private Autos() {
  }
}
