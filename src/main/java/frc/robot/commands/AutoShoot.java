package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntermediateSubsystem;

public class AutoShoot extends Command{
    private final ShooterSubsystem m_shoot;
    DigitalInput m_limitSwitch;
    public AutoShoot(ShooterSubsystem m_shoot){
        this.m_shoot = m_shoot;
        addRequirements(m_shoot);
    }

    public void execute(){
        m_shoot.shoot(0.7);
    }

    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted) {
        m_shoot.noRunshoot();
      }

}