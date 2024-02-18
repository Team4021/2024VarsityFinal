package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipConstants;

public class IntermediateSubsystem extends SubsystemBase{

    CANSparkMax m_intermediate = new CANSparkMax(ManipConstants.kIntermediate, MotorType.kBrushless);
    CANSparkMax m_intermediate2 = new CANSparkMax(ManipConstants.kIntermediate2, MotorType.kBrushless);

    public void configIntermediateMotors(){
        m_intermediate.restoreFactoryDefaults();
        m_intermediate2.restoreFactoryDefaults();

        //put configs here
        m_intermediate.setIdleMode(IdleMode.kBrake);
        m_intermediate2.setIdleMode(IdleMode.kBrake);

        m_intermediate.burnFlash();
        m_intermediate2.burnFlash();
    }

    public void noRunIntermediate(){

        m_intermediate.set(0);
        m_intermediate2.set(0);

    }

    public void runIntermediate(double speed){

        m_intermediate.set(speed);
        m_intermediate2.set(-speed);
        
    }



    
}
