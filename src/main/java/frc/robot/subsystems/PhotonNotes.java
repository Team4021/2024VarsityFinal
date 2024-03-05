package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonNotes extends SubsystemBase{

    PhotonCamera camera= new PhotonCamera("PhotonNotes");
    // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-notes");
    // NetworkTableEntry tx = table.getEntry("tx");
    // NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ta = table.getEntry("ta");
    // NetworkTableEntry tv = table.getEntry("tv");

    

    public double kP = -0.025;
    private DigitalInput m_limitSwitch;
    public boolean linedUp = false;
    PhotonTrackedTarget target;
    public boolean note;

    public PhotonNotes() {
    }

    @Override
    public void periodic(){
        var result = camera.getLatestResult();
        // List<PhotonTrackedTarget> targets = result.getTargets();
        target = result.getBestTarget();
        note = result.hasTargets();
    }

    public double getX(){
        double x = target.getYaw();
        return x;
    }
    public double getY(){
        double y = target.getPitch();
        return y;
    }
    public double getArea(){
        double area = target.getArea();
        return area;
    }
    // public boolean isNote(){
        // if (tv.getInteger(0)==0){
        //     note = false;
        // } else {
        //     note = true;
        // }
        
        // return note;
        // }

    public void defaultCommand(){
        // SmartDashboard.putNumber("XValue", getX());
        SmartDashboard.putBoolean("isNote?", note);
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
        if (/*note==true && */ m_limitSwitch.get()==false){
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


