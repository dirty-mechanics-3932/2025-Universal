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
    
    private double getSpeedFromTriggers() {
        double leftValue = m_driveController.getLeftTriggerAxis();
        double rightValue = m_driveController.getRightTriggerAxis();
        if (leftValue > 0.05) {
          return leftValue;
        }
        if (rightValue > 0.05) {
          return -rightValue;
        }
        return 0.0;
      }
    
    @Override
    public String robotName() {
        return "DarrylMini";
    }

    @Override
    public void robotInit() {
        Command darrylMoveBack = Commands.run(() -> m_dmotor.setSpeed(getSpeedFromTriggers()), m_dmotor);
        darrylMoveBack.ignoringDisable(true).schedule();
    }
}
