package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntermediateSubsystem;

public class Shoot extends Command{
    private final ShooterSubsystem m_shoot;
    private final IntermediateSubsystem m_inter;
    private final IntakeSubsystem m_intake;
    public Shoot(ShooterSubsystem m_shoot, IntermediateSubsystem m_inter, IntakeSubsystem m_intake){
        this.m_shoot = m_shoot;
        this.m_intake = m_intake;
        this.m_inter = m_inter;

        addRequirements(m_shoot, m_intake, m_inter);
    }

    public void execute(){

        // new RunCommand(() -> m_shoot.shoot(0.8), m_shoot)
        // .withTimeout(0.3)
        // .andThen(new RunCommand(() -> m_inter.runIntermediate(1), m_inter))
        // .andThen(new RunCommand(() -> m_intake.runIntake(0.3), m_intake));

        m_shoot.shoot(1);

        m_inter.runIntermediate(1);
        m_intake.runIntake(0.3);

    }
    public boolean isFinished(){
        return false;
    }
    public void end(){
        m_shoot.noRunshoot();
        m_intake.noRunIntake();
        m_inter.noRunIntermediate();
    }
}
