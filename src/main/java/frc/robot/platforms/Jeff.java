package frc.robot.platforms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.MotorSRX;
import frc.robot.utilities.Util;

public class Jeff implements RobotRunnable {
    private CommandXboxController m_controller;
    private MotorSRX m_redMotor;
    private CommandXboxController m_driveHID;
    
    public Jeff(CommandXboxController controller) {
        m_controller = controller;
        m_driveHID = controller;
        this.m_redMotor = new MotorSRX("redMotor", 2, 3, true);
        this.m_redMotor.invertFollow(true);
        Command redMoveCmd = Commands.run(() ->
        m_redMotor.setSpeed(Util.getSpeedFromTriggers(m_driveHID)), m_redMotor);
            
        new ScheduleCommand(redMoveCmd.ignoringDisable(true)).schedule();
    }

     @Override
    public String robotName(){
    return "Jeff";
    }

         @Override
    public void robotInit() {
        
        //Command JeffMove = Commands.run(() ->
            //m_redMotor.setSpeed(m_driveHID.getLeftTriggerAxis()), m_redMotor);
        //  m_driveHID.start().onTrue(backwardRedMoveCmd);
        //  m_driveHID.start().onTrue(redMoveCmd);
        // new ScheduleCommand(JeffMove);
    }
}
