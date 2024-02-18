package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipConstants;

public class ShooterSubsystem extends SubsystemBase {

    CANSparkMax m_shootMotor = new CANSparkMax(ManipConstants.kShootMotor, MotorType.kBrushless);
    CANSparkMax m_shootMotor2 = new CANSparkMax(33, MotorType.kBrushless);
    TalonFX m_angleMotor = new TalonFX(ManipConstants.kWristMotor);
    PositionVoltage m_request;
    


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
    public void shoot(double speed){
        m_shootMotor.set(-speed);
        m_shootMotor2.set(speed);
    }
    public void angleShooterClose(){
        m_request = new PositionVoltage(-8.9);
        m_angleMotor.setControl(m_request);
    }
    public void angleShooterFar(){
        m_request = new PositionVoltage(33);
        m_angleMotor.setControl(m_request);

    }
  
}
