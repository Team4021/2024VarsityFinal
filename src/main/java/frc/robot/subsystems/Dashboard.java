package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase{
    DigitalInput m_limitSwitch;

    public Dashboard(DigitalInput limitSwtich){
        this.m_limitSwitch = limitSwtich;
    }
    
    public void pushToDashboard(){
        SmartDashboard.putBoolean("Note In Intake", m_limitSwitch.get());
        
    }
}
