package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipConstants;;

public class IntakeSubsystem extends SubsystemBase {

    CANSparkFlex m_intakeMotor = new CANSparkFlex(ManipConstants.kIntakeMotor, MotorType.kBrushless);
    // private SparkPIDController m_intakePIDController;
    public double m_intakeSpeed;

    public void configIntakeMotor(){
        m_intakeMotor.restoreFactoryDefaults();
        m_intakeMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        // m_intakePIDController = m_intakeMotor.getPIDController();
        // m_intakePIDController.setP(ManipConstants.kIntakeP);
        // m_intakePIDController.setI(ManipConstants.kIntakeI);
        // m_intakePIDController.setD(ManipConstants.kIntakeD);
        // m_intakePIDController.setFF(ManipConstants.kIntakeFF);
        // m_intakePIDController.setOutputRange(ManipConstants.kIntakeMinOutput, ManipConstants.kIntakeMaxOutput);
        m_intakeMotor.burnFlash();
    }
    public void noRunIntake(){
        // m_intakePIDController.setReference(0, ControlType.kVelocity);
        SmartDashboard.putBoolean("testintake", false);
        m_intakeMotor.set(0);
        m_intakeSpeed = (0.0);

    }
    public void runIntake(){
        // m_intakePIDController.setReference(0.5, ControlType.kVelocity);
        SmartDashboard.putBoolean("testintake", true);
        m_intakeMotor.set(0.3);
        m_intakeSpeed = (0.3);
    }
    public void reverseIntake(){
        // m_intakePIDController.setReference(-0.5, ControlType.kVelocity);
        m_intakeMotor.set(-0.3);
        m_intakeSpeed = (-0.3);
    }
}
