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
  private double shoulderGearRatio = 100;
  private double elbowGearRatio = 60;
  private double deltaRotationsArm = 0.01; // Amount in rotations to move upon each press of the POV button
  private double deltaRotationsShoulder = 0.01; // Amount in rotations to move upon each press of the POV button
  private onTrueOnly pUp;
  private onTrueOnly pRight;
  private onTrueOnly pDown;
  private onTrueOnly pLeft;
  private boolean motionMagic = true;

  public BlondeMini(CommandXboxController controller) {
    // TODO Shoulder is 20 Elbow is 34 on Keith Mini Shoulder 10 Elbow 11
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
    PID positionPIDShoulder = new PID("Pos", 1.0, 0, 0, 0, 0, -1, 1, false);
    motorSparkMaxShoulder.putPIDtoMotor(positionPIDShoulder, ClosedLoopSlot.kSlot0);
    motorSparkMaxShoulder.setEncoderPosition(0);
    motorSparkMaxShoulder.setPos(0);
    motorSparkMaxShoulder.setCurrentLimit(30);
    if (motionMagic) { // Does not work yet
      PID motionMagicPIDShoulder = new PID("MotionMagic", 0.6, 0, 0, 0, 0, -1, 1, false);
      motorSparkMaxShoulder.putPIDtoMotor(motionMagicPIDShoulder, ClosedLoopSlot.kSlot2);
      motorSparkMaxShoulder.setVelocityAccelerationError(300, 500, .025, ClosedLoopSlot.kSlot2);
      motorSparkMaxElbow.putPIDtoMotor(motionMagicPIDShoulder, ClosedLoopSlot.kSlot2);
      motorSparkMaxElbow.setVelocityAccelerationError(300, 500, .025, ClosedLoopSlot.kSlot2);
    }

    PID positionPIDElbow = new PID("Pos", 0.16, 0, 0, 0, 0, -1, 1, false);
    motorSparkMaxElbow.putPIDtoMotor(positionPIDElbow, ClosedLoopSlot.kSlot0);
    motorSparkMaxElbow.setEncoderPosition(0);
    motorSparkMaxElbow.setPos(0);
    motorSparkMaxElbow.setCurrentLimit(20);
  }

  @Override
  public String robotName() {
    return "BlondeMini";
  }

  void setZero() {
    motorSparkMaxElbow.setEncoderPosition(0);
    motorSparkMaxElbow.setPos(0);
    lastElbow  = 0;
    motorSparkMaxShoulder.setEncoderPosition(0);
    motorSparkMaxShoulder.setPos(0);
    lastShoulder = 0;
    logf("Set Zero\n");
  }

  @Override
  public void teleopPeriodic() {
    count++;
    if (controller.a().getAsBoolean()) {
      setZero();
    }
    if (count % 250 == 0) {
      // Print log every 250 cycles or 5 seconds
      logf("Blond Mini Shoulder Cur:%.2f Pos:%.2f Elbow Cur:%.2f Pos:%.2f\n",
          motorSparkMaxShoulder.getMotorCurrent(),
          motorSparkMaxShoulder.getPos(),
          motorSparkMaxElbow.getMotorCurrent(),
          motorSparkMaxElbow.getPos());
    }
    if (count % 500 == 0) {
      // Print log every 250 cycles or 5 seconds
      logf("Blond Mini Count:%d Last Shoulder:%.2f Last Elbow:%.2f\n", count, lastShoulder, lastElbow);
    }
    if (motionMagic) {
      doMotionMagic();
    } else {
      doPosition();
    }
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

  // More the shoulder or elbow motors
  // a fixed delta rotation based when POV buttons are hit
  public void doPosition() {
    if (pUp.get()) {
      lastShoulder -= deltaRotationsShoulder * shoulderGearRatio;
      motorSparkMaxShoulder.setPos(lastShoulder);
    }
    if (pDown.get()) {
      lastShoulder += deltaRotationsShoulder * shoulderGearRatio;
      motorSparkMaxShoulder.setPos(lastShoulder);
    }
    if (pRight.get()) {
      lastElbow -= deltaRotationsArm * elbowGearRatio;
      motorSparkMaxElbow.setPos(lastElbow);
    }
    if (pLeft.get()) {
      lastElbow += deltaRotationsArm * elbowGearRatio;
      motorSparkMaxElbow.setPos(lastElbow);
    }
  }

  // More the shoulder or elbow motors
  // a fixed delta rotation based when POV buttons are hit
  public void doMotionMagic() {
    if (pUp.get()) {
      lastShoulder -= deltaRotationsShoulder * shoulderGearRatio;
      motorSparkMaxShoulder.setPosMotionMagic(lastShoulder);
    }
    if (pDown.get()) {
      lastShoulder += deltaRotationsShoulder * shoulderGearRatio;
      motorSparkMaxShoulder.setPosMotionMagic(lastShoulder);
    }
    if (pRight.get()) {
      lastElbow -= deltaRotationsArm * elbowGearRatio;
      motorSparkMaxElbow.setPosMotionMagic(lastElbow);
    }
    if (pLeft.get()) {
      lastElbow += deltaRotationsArm * elbowGearRatio;
      motorSparkMaxElbow.setPosMotionMagic(lastElbow);
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
    if (Math.abs(trig) < .03) {
      return;
    }
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
