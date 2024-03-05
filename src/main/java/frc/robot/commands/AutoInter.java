package frc.robot.commands;

import frc.robot.subsystems.IntermediateSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoInter extends Command{
    private final IntermediateSubsystem m_Inter;
    DigitalInput m_limitSwitch;
    public AutoInter(IntermediateSubsystem m_Inter){
        this.m_Inter = m_Inter;
        addRequirements(m_Inter);
    }

    public void execute(){
        m_Inter.runIntermediate(0.7);
    }

    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted) {
        m_Inter.noRunIntermediate();
      }

}