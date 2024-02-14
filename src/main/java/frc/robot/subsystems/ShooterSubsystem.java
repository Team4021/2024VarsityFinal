package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipConstants;

public class ShooterSubsystem extends SubsystemBase {

    CANSparkMax m_shootMotor = new CANSparkMax(ManipConstants.kShootMotor, MotorType.kBrushless);
    CANSparkMax m_shootMotor2 = new CANSparkMax(33, MotorType.kBrushless);
    CANSparkMax m_intermediate = new CANSparkMax(35, MotorType.kBrushless);


    // private SparkPIDController m_shootPIDController;

    public void configShootMotor(){
        // m_shootMotor.restoreFactoryDefaults();
        // m_shootMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        // m_shootPIDController = m_shootMotor.getPIDController();
        // m_shootPIDController.setP(ManipConstants.kShootP);
        // m_shootPIDController.setI(ManipConstants.kShootI);
        // m_shootPIDController.setD(ManipConstants.kShootD);
        // m_shootPIDController.setFF(ManipConstants.kShootFF);
        // m_shootPIDController.setOutputRange(ManipConstants.kShootMinOutput, ManipConstants.kShootMaxOutput);
        // m_shootMotor.burnFlash();
    }
    public void noRunshoot(){
        // m_shootPIDController.setReference(0, ControlType.kVelocity);
        m_shootMotor.set(0);
        m_shootMotor2.set(0);
        m_intermediate.set(0);
    }
    public void runAmp(){
        // m_shootPIDController.setReference(0.5, ControlType.kVelocity);
    }
    public void runSpeaker(){
        // m_shootPIDController.setReference(0, ControlType.kVelocity);
    }
    public void reverseShoot(){
        // m_shootPIDController.setReference(-0.5, ControlType.kVelocity);
    }
    public void shoot(){
        m_shootMotor.set(-0.6);
        m_shootMotor2.set(0.6);
    }
    public void intermediate(){
        m_intermediate.set(0.6);
    }
}
