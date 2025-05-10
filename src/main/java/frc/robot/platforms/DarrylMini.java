package frc.robot.platforms;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSRX;
import frc.robot.subsystems.MotorSRX;

public class DarrylMini implements RobotRunnable {
    
    MotorSRX dmotor;
    final XboxController driveHID;
    final CommandXboxController driveController;
    public DarrylMini() {
        
        driveController = new CommandXboxController(2);
        driveHID = driveController.getHID();
        dmotor = new MotorSRX("DarrylSRX", 10, -1, true);
        new DrivetrainSRX(driveHID);
    }
    private double getSpeedFromTriggers() {
        double leftValue = driveController.getLeftTriggerAxis();
        double rightValue = driveController.getRightTriggerAxis();
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
        return RobotRunnable.super.robotName();
    }
    @Override
    public void robotInit() {
        Command darrylMoveBack = Commands.run(() -> dmotor.setSpeed(getSpeedFromTriggers()), dmotor);
        darrylMoveBack.ignoringDisable(true).schedule();
        RobotRunnable.super.robotInit();
    }
}
