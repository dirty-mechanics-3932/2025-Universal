package frc.robot.platforms;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DrivetrainSpark;

public class Mando implements RobotRunnable {

    public Mando(XboxController driveHID) {
        new DrivetrainSpark(driveHID);
    }

    @Override
    public void teleopPeriodic() {

    }
}
