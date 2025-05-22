package frc.robot.platforms;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSRX;
import frc.robot.subsystems.MotorSRX;

public class DarrylMini implements RobotRunnable {
    
    MotorSRX m_dmotor;
    final XboxController m_driveHID;
    final CommandXboxController m_driveController;
    
    public DarrylMini() {
        m_driveController = new CommandXboxController(2);
        m_driveHID = m_driveController.getHID();
        m_dmotor = new MotorSRX("DarrylSRX", 10, -1, true);
        new DrivetrainSRX(m_driveHID);
    }
    
    @Override
    public String robotName() {
        return "DarrylMini";
    }

    @Override
    public void robotInit() {
        Command darrylMoveBack = Commands.run(() -> m_dmotor.setSpeed(getTriggerValue(m_driveController)), m_dmotor);
        darrylMoveBack.ignoringDisable(true).schedule();
    }
}
