package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSubsystem extends SubsystemBase {

    NetworkTable m_notes;
    NetworkTable m_tags;

    private final Limelight m_noteTacker = new Limelight("limelight1");
    private final Limelight m_tagTracker = new Limelight("limelight2");

    public LimelightSubsystem() {

    }

}
