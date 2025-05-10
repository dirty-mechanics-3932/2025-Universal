package frc.robot.platforms;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.MotorFlex;
import frc.robot.subsystems.MotorSRX;
import frc.robot.subsystems.PID;

public class MiniMini implements RobotRunnable {
    private MotorSRX m_redMotor;
    private MotorFlex m_flexMotor;
    private CommandXboxController m_driveHID;

    public MiniMini(int redMotorCanID, int flexMotorCanID, CommandXboxController driverHID) {
        m_redMotor = new MotorSRX("RedMotor", redMotorCanID, -1, true);
        PID positionPID = new PID("Pos", .08, 0, 0, 0, 0, -1, 1, true);
        PID velocityPID = new PID("Vel", .005, 0, 0, 0, 1.5, -1, 1, true);
        // Motion Magic messes things up positionPID.setMotionMagicSRX(.5, 2.0);
        m_redMotor.setPositionPID(positionPID, 0, FeedbackDevice.QuadEncoder); // set pid for SRX
        m_redMotor.setVelocityPID(velocityPID, 1, FeedbackDevice.QuadEncoder);
    
        m_flexMotor = new MotorFlex("FlexMotor", flexMotorCanID, -1, true);
        m_flexMotor.setLogging(true);
        m_flexMotor.setTestMode(true);
        // redMotor.setUpForTestCases(leds);
        // redMotor.setLogging(true);
        // redMotor.setEncoderTicksPerRev(2048);

//         MotorFlex flexMotor = new MotorFlex("FlexMotor", 3, -1, true);
//         flexMotor.setLogging(true);
//         flexMotor.setTestMode(true);
//         redMotor.setUpForTestCases(leds);
//         redMotor.setLogging(true);
//         redMotor.setEncoderTicksPerRev(2048);
//         Command redMoveCmd = Commands.run(() ->
//         redMotor.setSpeed(driveController.getLeftTriggerAxis()), redMotor);
//         Command neoMoveCmd = Commands.run(() ->
//         flexMotor.setSpeed(driveController.getRightTriggerAxis()), flexMotor);
One change
//         new ScheduleCommand(Commands.parallel(redMoveCmd,
//         neoMoveCmd).ignoringDisable(true)).schedule();
//         Command miniMove = Commands.run(() ->
//         flexMotor.setSpeed(driveController.getLeftTriggerAxis()), flexMotor);
//         driveController.start().onTrue(miniMove);
//         new ScheduleCommand(miniMove);
    }

    @Override
    public void robotInit() {
        Command redMoveCmd = Commands.run(() ->
            m_redMotor.setSpeed(m_driveHID.getLeftTriggerAxis()), m_redMotor);
        Command neoMoveCmd = Commands.run(() ->
            m_flexMotor.setSpeed(m_driveHID.getRightTriggerAxis()), m_flexMotor);
        new ScheduleCommand(Commands.parallel(redMoveCmd, neoMoveCmd).ignoringDisable(true)).schedule();
        Command miniMove = Commands.run(() ->
            m_flexMotor.setSpeed(m_driveHID.getLeftTriggerAxis()), m_flexMotor);
        m_driveHID.start().onTrue(miniMove);
        new ScheduleCommand(miniMove);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
