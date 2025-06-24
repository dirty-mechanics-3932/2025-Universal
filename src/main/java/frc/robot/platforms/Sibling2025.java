package frc.robot.platforms;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DrivetrainTestSwerve;

public class Sibling2025 implements RobotRunnable {
    //final private DrivetrainTestSwerve m_swerve;
    //final private XboxController m_driveHID;

    public Sibling2025(XboxController driveHID) {
       // m_driveHID = driveHID;
        new DrivetrainTestSwerve(driveHID);
    }
    
}
