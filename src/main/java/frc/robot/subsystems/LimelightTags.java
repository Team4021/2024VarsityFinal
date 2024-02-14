package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightTags extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-tags");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");


    public double getX(){
        double x = tx.getDouble(0.0);
        return x;
    }
    public double getY(){
        double y = ty.getDouble(0.0);
        return y;
    }
    public double getArea(){
        double area = ta.getDouble(0.0);
        return area;
    }
    public boolean isTag(){
        boolean note;
        if (tv.getInteger(0)==0){
            note = false;
        } else {
            note = true;
        }
        return note;
    }

}
