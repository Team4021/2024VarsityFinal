package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightNotes;
import frc.robot.subsystems.PhotonNotes;

public class TrackingIntake extends Command{
    DriveSubsystem m_robotDrive;
    LimelightNotes m_limelightNotes;
    Double xSpeed;
    Double ySpeed;
    Double rot;
    DigitalInput m_limitSwitch;
    private final CommandJoystick m_strafeController =
        new CommandJoystick(OIConstants.kStrafeControllerPort);
    private final CommandJoystick m_turnController =
        new CommandJoystick(OIConstants.kTurnControllerPort);
    PhotonNotes m_photonNotes;

    public TrackingIntake(DriveSubsystem robotDrive, PhotonNotes photonNotes, DigitalInput limitSwitch){
        this.m_robotDrive = robotDrive;
        this.m_photonNotes = photonNotes;
        this.m_limitSwitch = limitSwitch;

        addRequirements(m_robotDrive, m_photonNotes);
    }

    public void execute(){
  
        m_robotDrive.drive(
            MathUtil.applyDeadband(m_strafeController.getY(), OIConstants.kJoystickDeadband), 
            m_photonNotes.changeXspeed(
            MathUtil.applyDeadband(m_strafeController.getX(), OIConstants.kJoystickDeadband), m_limitSwitch), 
            -MathUtil.applyDeadband(m_turnController.getX(), OIConstants.kJoystickDeadband),  
            false, true);
    }

    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted){
        m_limelightNotes.linedUp = false;
    }
}
