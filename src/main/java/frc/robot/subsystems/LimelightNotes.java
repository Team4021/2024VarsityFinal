package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightNotes extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-notes");
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
    public boolean isNote(){
        boolean note;
        if (tv.getInteger(0)==0){
            note = false;
        } else {
            note = true;
        }
        return note;
    }

    public void defaultCommand(){
        SmartDashboard.putNumber("XValue", getX());
        SmartDashboard.putBoolean("isNote?", isNote());
    }

    public double changeXspeed(double inputSpeed){

        double outputSpeed;
        if (isNote()==true){
            if (getX()<-10){
                outputSpeed = inputSpeed-1;
            } else if (getX()>10){
                outputSpeed = inputSpeed+1;
            } else{
                outputSpeed = inputSpeed;
            }
        } else{
            outputSpeed = inputSpeed;
        }

        return outputSpeed;
    }

}
