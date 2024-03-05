package frc.robot.subsystems;

import java.lang.invoke.VolatileCallSite;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    
    TalonFX m_leftClimb = new TalonFX(41);
    TalonFX m_rightClimb = new TalonFX(42);
    PositionDutyCycle up = new PositionDutyCycle(1);
    PositionDutyCycle down = new PositionDutyCycle(-1);
    

    public ClimberSubsystem(){
    }
    public void no(){
        m_leftClimb.stopMotor();
        m_rightClimb.stopMotor();
    }

    public void up(){
        // m_leftClimb.setControl(up);
        // m_rightClimb.setControl(up);
        m_leftClimb.set(-1);
        m_rightClimb.set(1);
    }

    public void down(){
        // m_leftClimb.setControl(down);
        // m_rightClimb.setControl(down);
        m_leftClimb.set(1);
        m_rightClimb.set(-1);
    }

    

}
