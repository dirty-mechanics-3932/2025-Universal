package frc.robot.platforms;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DrivetrainSRX;
import frc.robot.subsystems.MotorSparkMax;

public class BlondeMini implements RobotRunnable {
    private final DrivetrainSRX drivetrainSRX;
    private final MotorSparkMax motorSparkMax;
    private final XboxController hid;

    public BlondeMini(XboxController hid){
        motorSparkMax = new MotorSparkMax("TestMax", 20, -1, false, false); 
        drivetrainSRX = new DrivetrainSRX(hid); 
        this.hid = hid;
    }
    @Override
    public String robotName() {
        return "BlondeMini";
    }

    @Override
    public void robotInit() {
        Command blondeMove = Commands.run(() -> motorSparkMax.setSpeed(getSpeedFromTriggers()), motorSparkMax);
        blondeMove.ignoringDisable(true).schedule();
    }

    @Override 
    public void testInit() {
        motorSparkMax.setLogging(true);
        motorSparkMax.setTestMode(true);
    }

    private double getSpeedFromTriggers() {
        double leftValue = hid.getLeftTriggerAxis();
        double rightValue = hid.getRightTriggerAxis();
        if (leftValue > 0.05) {
          return leftValue;
        }
        if (rightValue > 0.05) {
          return -rightValue;
        }
        return 0.0;
      }
}
