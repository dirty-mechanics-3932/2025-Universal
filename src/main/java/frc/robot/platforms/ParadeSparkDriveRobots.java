package frc.robot.platforms;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DrivetrainSpark;

//used for train
public class ParadeSparkDriveRobots implements RobotRunnable {
     private String robotName; 
     public ParadeSparkDriveRobots(XboxController hid, String robotName){
       new DrivetrainSpark(hid); 
       
        this.robotName = robotName;
    }
    
    @Override
    public String robotName() {
        return robotName; 
    }
    @Override
    public void teleopPeriodic() {
        
    }
}
