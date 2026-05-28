package frc.robot.platforms;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSRX;
import frc.robot.subsystems.MotorSRX;

public class MiniRaph implements RobotRunnable {
   
    MotorSRX m_rmotor;
    final XboxController m_driveHID;
    final CommandXboxController m_driveController;

    public MiniRaph() {
        m_driveController = new CommandXboxController(2);
        m_driveHID = m_driveController.getHID();
        m_rmotor = new MotorSRX("RaphSRX", 11, -1, m_driveController, true);
        new DrivetrainSRX(m_driveHID);
    }

    @Override
    public String robotName() {
        return "MiniRaph";
    }

    @Override
    public void robotInit() {
        Command raphMoveBack = Commands.run(() -> m_rmotor.setSpeed(getTriggerValue(m_driveController)), m_rmotor);
        raphMoveBack.ignoringDisable(true).schedule(); 

}
}

