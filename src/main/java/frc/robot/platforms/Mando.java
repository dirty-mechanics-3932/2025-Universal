package frc.robot.platforms;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DrivetrainSpark;

public class Mando implements RobotRunnable {
    private final DrivetrainSpark m_drivetrain; 
    private final XboxController m_driveHID;

    public Mando(XboxController driveHID) {
        m_driveHID = driveHID;
        m_drivetrain = new DrivetrainSpark(driveHID);
    }
}
