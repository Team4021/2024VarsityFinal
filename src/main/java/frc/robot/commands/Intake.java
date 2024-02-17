package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntermediateSubsystem;

public class Intake extends Command{
    // private DigitalInput m_limitSwitch = new DigitalInput(0);
    private final IntermediateSubsystem m_inter;
    private final IntakeSubsystem m_intake;
    public Intake(IntermediateSubsystem m_inter, IntakeSubsystem m_intake){
        this.m_intake = m_intake;
        this.m_inter = m_inter;

        addRequirements(m_intake, m_inter);
    }

    public void execute(){
        // if (m_limitSwitch.get() == false){
            m_intake.runIntake(0.3);
            m_inter.runIntermediate(0.3);
        // } else if (m_limitSwitch.get() == true){
        //     m_intake.noRunIntake();
        //     m_inter.noRunIntermediate();
        // }
    }

    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted) {
        m_inter.noRunIntermediate();
        m_intake.noRunIntake();
      }

}