package frc.robot.platforms;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DrivetrainSRX;

//used for kevin and squidward
public class ParadeSrxDriveRobots implements RobotRunnable {
     //private final DrivetrainSRX drivetrainSRX;
     //private final XboxController hid; 
     private String robotName; 
     public ParadeSrxDriveRobots(XboxController hid, String robotName){
       new DrivetrainSRX(hid); 
       
        this.robotName = robotName;
    }
    
    @Override
    public String robotName() {
        return robotName; 
    }
}
