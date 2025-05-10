package frc.robot.platforms;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DrivetrainSRX;

//used for kevin and squidward
public class ParadeSrxDriveRobots implements RobotRunnable {
     private final DrivetrainSRX drivetrainSRX;
     private final XboxController hid; 
     public ParadeSrxDriveRobots(XboxController hid){
        drivetrainSRX = new DrivetrainSRX(hid); 
        this.hid = hid;
    }
    
}
