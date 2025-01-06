package frc.robot.vision;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PiCamera {
    
    private final DoubleSubscriber angleSub;

    public PiCamera(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // get the subtable called "datatable"
        NetworkTable datatable = inst.getTable("SmartDashboard");
    
        // subscribe to the topic in "datatable" called "Y"
        angleSub = datatable.getDoubleTopic("anglex").subscribe(0.0);
    }

    public double getPiCamAngle(){
        return angleSub.get();
    }
}
