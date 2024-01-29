package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipConstants;



public class WristSubsystem extends SubsystemBase{

    CANSparkFlex m_wristMotor = new CANSparkFlex(ManipConstants.kWristMotor, MotorType.kBrushless);

    private SparkPIDController m_wristPIDController;



    public void configWristMotor(){
        m_wristMotor.restoreFactoryDefaults();
        m_wristMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        m_wristPIDController = m_wristMotor.getPIDController();
        m_wristPIDController.setP(ManipConstants.kWristP);
        m_wristPIDController.setI(ManipConstants.kWristI);
        m_wristPIDController.setD(ManipConstants.kWristD);
        m_wristPIDController.setFF(ManipConstants.kWristFF);
        m_wristPIDController.setOutputRange(ManipConstants.kWristMinOutput, ManipConstants.kWristMaxOutput);
        m_wristMotor.burnFlash();
    }

    public void setHome(){
        m_wristPIDController.setReference(0, ControlType.kPosition);
    }
    public void setAmp(){
        m_wristPIDController.setReference(0, ControlType.kPosition);
    }
}
