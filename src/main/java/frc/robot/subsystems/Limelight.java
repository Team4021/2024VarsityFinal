package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    double lmlx;
    double lmly;
    double lmla;
    double lmlv;
    
    public Limelight(String limelight){
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelight);

        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");

    }

    public double getX(){

        double x;
        
        NetworkTableEntry tx = table.getEntry("tx");
        x = tx.getDouble(0.0);
        return x;

    }

}
