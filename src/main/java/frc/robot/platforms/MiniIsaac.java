package frc.robot.platforms;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.DrivetrainSRX;
import frc.robot.subsystems.MotorFlex;
import frc.robot.subsystems.MotorSRX;
import frc.robot.subsystems.PID;

public class MiniIsaac implements RobotRunnable {

    private final CommandXboxController m_driveController;
    MotorFlex neoMotor;
    MotorSRX redMotor2;
    SparkMaxConfig motorConfig;
    PID neoPIDMotionMagic;
    Command turnNeoMotor;

    public MiniIsaac() {

        m_driveController = new CommandXboxController(2);

        // neoMotor = new MotorFlex("NeoMotor", 3, -1, true);
        redMotor2 = new MotorSRX("RedMotor", 10, -1, m_driveController, true);
        motorConfig = new SparkMaxConfig();
        // neoPIDMotionMagic = new PID("neoMotorPID", 1, 0, 0, 0, 0, -1, 1, false);

    }

    private double getSpeedFromTriggers() {
        double leftValue = m_driveController.getLeftTriggerAxis();
        double rightValue = m_driveController.getRightTriggerAxis();
        if (leftValue > 0.05) {
            return leftValue;

        } else if (rightValue > 0.05) {
            return -rightValue;

        } else {
            return 0.0;
        }
    }

    @Override
    public String robotName() {
        return "MiniIsaac";
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

        neoMotor.setSpeed(getSpeedFromTriggers());

        m_driveController.a().whileTrue(redMotor2.sysIdDynamic(Direction.kForward));
        m_driveController.b().whileTrue(redMotor2.sysIdDynamic(Direction.kReverse));
        m_driveController.x().whileTrue(redMotor2.sysIdQuasistatic(Direction.kForward));
        m_driveController.y().whileTrue(redMotor2.sysIdQuasistatic(Direction.kReverse));
    }
}
