package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipConstants;

public class ShooterSubsystem extends SubsystemBase {

    DutyCycleEncoder m_absolute = new DutyCycleEncoder(1);
    CANSparkMax m_shootMotor = new CANSparkMax(ManipConstants.kShootMotor, MotorType.kBrushless);
    CANSparkMax m_shootMotor2 = new CANSparkMax(33, MotorType.kBrushless);
    TalonFX m_angleMotor = new TalonFX(ManipConstants.kWristMotor);
    PositionVoltage m_request;
    // SoftwareLimitSwitchConfigs m_angleLimitConfigs;
    // FeedbackConfigs m_angleFeedbackConfigs;
    


    // private SparkPIDController m_shootPIDController;

    public void configShootMotor(){
        m_shootMotor.restoreFactoryDefaults();
        m_shootMotor2.restoreFactoryDefaults();
        m_shootMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        m_shootMotor2.setIdleMode(CANSparkBase.IdleMode.kBrake);
        // m_shootPIDController = m_shootMotor.getPIDController();
        // m_shootPIDController.setP(ManipConstants.kShootP);
        // m_shootPIDController.setI(ManipConstants.kShootI);
        // m_shootPIDController.setD(ManipConstants.kShootD);
        // m_shootPIDController.setFF(ManipConstants.kShootFF);
        // m_shootPIDController.setOutputRange(ManipConstants.kShootMinOutput, ManipConstants.kShootMaxOutput);
        m_shootMotor.burnFlash();
        m_shootMotor2.burnFlash();

        // m_angleLimitConfigs.withForwardSoftLimitEnable(true);
        // m_angleLimitConfigs.withReverseSoftLimitEnable(true);
        // m_angleLimitConfigs.withForwardSoftLimitThreshold(0.8366);
        // m_angleLimitConfigs.withReverseSoftLimitThreshold(0.6466);
        // m_angleFeedbackConfigs.withRotorToSensorRatio(256);

        // m_angleMotor.getConfigurator().apply(m_angleLimitConfigs);
        // m_angleMotor.getConfigurator().apply(m_angleFeedbackConfigs);
        // m_angleMotor.getConfigurator().setPosition(m_absolute.getAbsolutePosition());

    }
    public void noRunshoot(){
        // m_shootPIDController.setReference(0, ControlType.kVelocity);
        m_shootMotor.set(0);
        m_shootMotor2.set(0);
        m_angleMotor.stopMotor();
        SmartDashboard.putNumber("absoluteangle", m_absolute.getAbsolutePosition());
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
        m_request = new PositionVoltage(0.6466);
        m_angleMotor.setControl(m_request);
    }
    public void angleShooterFar(){
        m_request = new PositionVoltage(0.8366);
        m_angleMotor.setControl(m_request);
    }
  
}
