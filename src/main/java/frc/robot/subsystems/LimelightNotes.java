package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightNotes extends SubsystemBase{

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-notes");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    public double kP = -0.025;
    private DigitalInput m_limitSwitch;
    public boolean linedUp = false;

    public LimelightNotes() {
    }
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
        // SmartDashboard.putNumber("XValue", getX());
        SmartDashboard.putBoolean("isNote?", isNote());
        // linedUp = false;
    }

    // public double changeYSpeed(double inputSpeed){
    //     double outputYSpeed;
    //     if (isNote()==true){
    //         if (getX()<-10){
    //             outputYSpeed = 0.5;
    //         } else if (getX()>10){
    //             outputYSpeed = 0.5;
    //         } else{
    //             outputYSpeed = 1;
    //         }
    //     } else{
    //         outputYSpeed = inputSpeed;
    //     }

    //     return outputYSpeed;
    // }

    public double changeXspeed(double inputSpeed, DigitalInput limitSwtich){
        double outputXSpeed;

        SmartDashboard.putBoolean("linedUp", linedUp);
        SmartDashboard.putNumber("x", getX());
        this.m_limitSwitch =  limitSwtich;
        if (isNote()==true && m_limitSwitch.get()==false){
            if (linedUp==false){
                if (getX()<-0.5){
                    outputXSpeed = getX() * kP;

                } else if (getX()>0.5){
                    outputXSpeed = getX() * kP;
                } else{
                    outputXSpeed = 0;
                    linedUp = true;
                }
            } else {
                outputXSpeed = 0;
            }
        // } else if (m_limitSwitch.get()==true){
        //     outputXSpeed = inputSpeed;
        //     linedUp = false;
        }
        else{
            outputXSpeed = inputSpeed;
        }

        return outputXSpeed;
    }
    public void noLinedUp(){
        linedUp = false;
    }

}
