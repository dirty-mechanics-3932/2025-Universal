package frc.robot.platforms;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DrivetrainSRX;

//used for kevin and squidward
public class ParadeSrxDriveRobots implements RobotRunnable { 

    private String robotName; 
    
     public ParadeSrxDriveRobots(XboxController hid, String robotName, DrivetrainSRX.DriveTrain type){
       new DrivetrainSRX(hid, type); 
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
