package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntermediateSubsystem;

public class Shoot extends Command{
    private final ShooterSubsystem m_shoot;
    private final IntermediateSubsystem m_inter;
    private final IntakeSubsystem m_intake;
    // private double target = 0.0;
    public double counter = 0.0;
    public Shoot(ShooterSubsystem m_shoot, IntermediateSubsystem m_inter, IntakeSubsystem m_intake){
        this.m_shoot = m_shoot;
        this.m_intake = m_intake;
        this.m_inter = m_inter;
        

        addRequirements(m_shoot, m_intake, m_inter);
    }
    public void initialize(){
        counter = 0.0;
    }

    public void execute(){

          counter++;
          m_shoot.shoot(0.7);
          if(counter > 30){
              m_inter.runIntermediate(0.7);
              m_intake.runIntake(0.3);
          }
        
      
          
    }
    public boolean isFinished(){
        return false;
    }
    public void end(Boolean interrupted){
        m_shoot.noRunshoot();
        m_intake.noRunIntake();
        m_inter.noRunIntermediate();
    }
}
