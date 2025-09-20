package frc.robot.platforms;

import static frc.robot.utilities.Util.logf;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DrivetrainSRX;
import frc.robot.subsystems.MotorSparkMax;
import frc.robot.subsystems.PID;

public class BlondeMini implements RobotRunnable {
  private final MotorSparkMax motorSparkMaxShoulder;
  private final MotorSparkMax motorSparkMaxElbow;
  private CommandXboxController controller;
  public int count = 0;
  private double lastShoulder = 0;
  private double lastElbow = 0;
  private double shoulderGearRatio = 20;
  private double elbowGearRatio = 60;
  private double deltaRotations = 0.05; // Amount in rotations to move upon each press of the POV button
  private onTrueOnly pUp;
  private onTrueOnly pRight;
  private onTrueOnly pDown;
  private onTrueOnly pLeft;

  public BlondeMini(CommandXboxController controller) {
    motorSparkMaxShoulder = new MotorSparkMax("Shoulder", 20, -1, controller, false, false);
    motorSparkMaxElbow = new MotorSparkMax("Elbow", 34, -1, controller, false, false);
    new DrivetrainSRX(controller.getHID());
    this.controller = controller;
    // Setup POV for geting transisiton from false to true
    pUp = new onTrueOnly(controller.povUp());
    pRight = new onTrueOnly(controller.povRight());
    pDown = new onTrueOnly(controller.povDown());
    pLeft = new onTrueOnly(controller.povLeft());

    // Setup PIDs the Shoulder motor needs for kP
    PID positionPIDShoulder= new PID("Pos", 0.25, 0, 0, 0, 0, -1, 1, false);
    PID positionPIDElbow = new PID("Pos", 0.16, 0, 0, 0, 0, -1, 1, false);
    motorSparkMaxShoulder.putPIDtoMotor(positionPIDShoulder, ClosedLoopSlot.kSlot0);
    motorSparkMaxElbow.putPIDtoMotor(positionPIDElbow, ClosedLoopSlot.kSlot0);
  }

  @Override
  public String robotName() {
    return "BlondeMini";
  }

  @Override
  public void teleopPeriodic() {
    count++;
    if (count % 250 == 0) {
      // Print log every 250 cycles or 5 seconds
      logf("Blond Mini Count:%d Last Shoulder:%.2f Last Elbow:%.2f\n", count, lastShoulder, lastElbow);
    }
    doPosition();
  }

  @Override
  public void robotInit() {
    Command blondeMove = Commands.run(() -> motorSparkMaxShoulder.setSpeed(getTriggerValue(controller)),
        motorSparkMaxShoulder);
    blondeMove.ignoringDisable(true).schedule();
  }

  @Override
  public void testInit() {
    motorSparkMaxShoulder.setLogging(true);
    motorSparkMaxShoulder.setTestMode(true);
  }

  // More the shoulder or elbow motors a fixed delta rotation based when POV buttons are hit
  public void doPosition() {
    if (pUp.get()) {
      lastShoulder += deltaRotations * shoulderGearRatio;
      motorSparkMaxShoulder.setPos(lastShoulder);
    }
    if (pDown.get()) {
      lastShoulder -= deltaRotations * shoulderGearRatio;
      motorSparkMaxShoulder.setPos(lastShoulder);
    }
    if (pRight.get()) {
      lastElbow += deltaRotations * elbowGearRatio;
      motorSparkMaxElbow.setPos(lastElbow);
    }
    if (pLeft.get()) {
      lastElbow -= deltaRotations * elbowGearRatio;
      motorSparkMaxElbow.setPos(lastElbow);
    }
  }

  // Class to determing when a trigger goes from False to True
  public class onTrueOnly {
    boolean last = false;
    Trigger trigger;

    private onTrueOnly(Trigger trigger) {
      this.trigger = trigger;
    }

    private boolean get() {
      boolean val = trigger.getAsBoolean();
      // Test to see if trigger has gone from false to true
      if (val && !last) {
        last = val;
        return true;
      }
      last = val;
      return false;
    }
  }

  // Can be used for testing the Position PID for the motors
  public void testPosition() {
    double trig = getTriggerValue(controller);
    if (Math.abs(trig) < .03)
      return;
    motorSparkMaxShoulder.setPos(trig * shoulderGearRatio);
    motorSparkMaxElbow.setPos(trig * elbowGearRatio);
  }

  // Can be used for testing the motor are working
  public void testSpeeds() {
    // Test Motor speed
    double trig = getTriggerValue(controller);
    motorSparkMaxShoulder.setSpeed(trig);
    motorSparkMaxElbow.setSpeed(trig);
  }
}
