package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utilities.Util.logf;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.utilities.Util;

// Motion Magic for Kraken has some code at
// https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/MotionMagic/src/main/java/frc/robot/Robot.java

public class MotorKraken extends SubsystemBase {
  private boolean myLogging = false;
  private boolean testMode = false;
  private String name = "";
  private CommandXboxController controller;
  private final TalonFX motor;
  private int numberCyclesForDisplay = 1000000;
  // Start at position 0, use slot 0 for Postion Control
  private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
  // Start at velocity 0, use slot 1 for Velocity Control
  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(1);
  // Start at position 0, use slot 2 for position motion magic
  private final MotionMagicVoltage smartPositionVoltage = new MotionMagicVoltage(0).withSlot(2);
  // Start at position 0, use slot 2 for position motion magic
  private final VoltageOut voltage = new VoltageOut(0);

  // private MotorKrakenInputsAutoLogged inputs = new
  // MotorKrakenInputsAutoLogged();
  private final SysIdRoutine sysId;

  @AutoLog
  public static class MotorKrakenInputs {
    public Angle position = Radians.zero();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVolts = Volts.zero();
    public Current currentSupplyAmps = Amps.zero();
    public Current currentStatorAmps = Amps.zero();
  }

  public MotorKraken(String name, int id, int followId, CommandXboxController controller, boolean logging) {
    this.controller = controller;
    this.name = name;
    // Keep a brake request so we can disable the motor
    // brake = new NeutralOut();
    motor = new TalonFX(id);
    configKraken(motor);
    /* Make sure we start the encoder at positon 0 */
    motor.setPosition(0);

    // Configure SysId
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            Time.ofBaseUnits(3.5, Seconds),
            (state) -> Logger.recordOutput(name + "/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage), null, this));
  }

  private void configKraken(TalonFX motor) {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    // Set slot 0 for position PID
    configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    configs.Slot0.kI = 0; // No output for integrated error
    configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    // Peak output of 8 V
    configs.Voltage.withPeakForwardVoltage(Volts.of(12))
        .withPeakReverseVoltage(Volts.of(-12));

    // * Voltage-based velocity requires a velocity feed forward
    configs.Slot1.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    configs.Slot1.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                             // volts / rotation per second
    configs.Slot1.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot1.kI = 0; // No output for integrated error
    configs.Slot1.kD = 0; // No output for error derivative

    /* Configure gear ratio */
    FeedbackConfigs fdb = configs.Feedback;
    fdb.SensorToMechanismRatio = 1; // 1 rotor rotations per mechanism rotation

    // Configure Motion Magic 5 (mechanism) rotations per second cruise
    configs.MotionMagic.withMotionMagicCruiseVelocity(RotationsPerSecond.of(20));
    // Take approximately 0.5 seconds to reach max vel
    configs.MotionMagic.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(50));
    // Take approximately 0.1 seconds to reach max accel
    // configs.MotionMagic.withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

    configs.Slot2.kS = 0.25; // Add 0.25 V output to overcome static friction
    configs.Slot2.kV = 2; // A velocity target of 1 rps results in 0.12 V output
    configs.Slot2.kA = 2; // An acceleration of 1 rps/s requires 0.01 V output
    configs.Slot2.kP = .5; // A position error of 0.2 rotations results in 12 V output
    configs.Slot2.kI = 0; // No output for integrated error
    configs.Slot2.kD = 0; // A velocity error of 1 rps results in 0.5 V output
    // configs.MotionMagic.withMotionMagicCruiseVelocity(10.0); // Reduced from 20.0
    // configs.MotionMagic.withMotionMagicAcceleration(20.0); // Reduced from 40.0
    // configs.MotionMagic.withMotionMagicJerk(2000.0); // Reduced from 4000.0

    StatusCode status = motor.getConfigurator().apply(configs);
    if (!status.isOK()) {
      logf("Unable to set status for Kraken motor:%s\n", name);
      return;
    }
  }

  public void setLogging(boolean value) {
    myLogging = value;
  }

  public void setTestMode(boolean value) {
    testMode = value;
  }

  // Set speed as a value from -1 to 1
  public void setSpeed(double value) {
    motor.set(value);
  }

  // Set the position of the motor to a value
  private void setPos(double value) {
    motor.setControl(positionVoltage.withPosition(value));
  }

  // Set the postion of the motor to a value using Magic Motion
  private void setPositionMotionMagic(double value) {
    motor.setControl(smartPositionVoltage.withPosition(value).withSlot(2));
  }

  private void setVelocity(double value) {
    /* Use velocity voltage */
    motor.setControl(velocityVoltage.withVelocity(value));
  }

  private void setVoltage(Voltage value) {
    /* Use velocity voltage */
    motor.setControl(voltage.withOutput(value));

  }

  public void setSmartTicks(int numberLoopsForDisplay) {
    if (numberLoopsForDisplay <= 0)
      this.numberCyclesForDisplay = Integer.MAX_VALUE;
    else
      this.numberCyclesForDisplay = numberLoopsForDisplay;
  }

  @Override
  public void periodic() {
    double err = motor.getClosedLoopError().getValueAsDouble();
    double pos = motor.getPosition().getValueAsDouble();
    double velocity = motor.getVelocity().getValueAsDouble();
    double current = motor.getStatorCurrent().getValueAsDouble();
    if (Robot.count % numberCyclesForDisplay == 0) {
      SmartDashboard.putNumber("Err", err);
      SmartDashboard.putNumber("Pos", pos);
      SmartDashboard.putNumber("Vel", velocity);
      SmartDashboard.putNumber("RPM", velocity * 60.0); // Get Velocity in PRM
      SmartDashboard.putNumber("Cur", current);
      SmartDashboard.putNumber("SetP", setP);
      SmartDashboard.putString("Mode", mode.toString());
    }
    if (Math.abs(velocity) > 0.05 && myLogging && Robot.count % 20 == 0) {
      logf("%s vel:%.2f RPM:%.2f cur:%.2f pos:%.2f err:%.2f\n", name, velocity, velocity * 60.0, current, pos, err);
    }
    if (testMode) {
      testCases();
    }

    /*
     * inputs.position = motor.getPosition().getValue();
     * inputs.velocity = motor.getVelocity().getValue();
     * inputs.appliedVolts = motor.getMotorVoltage().getValue();
     * inputs.currentStatorAmps = motor.getStatorCurrent().getValue();
     * inputs.currentSupplyAmps = motor.getSupplyCurrent().getValue();
     */
    // Logger.processInputs(name);
  }

  enum Modes {
    POSITION, VELOCITY, MOTIONMAGIC, SPEED;

    public Modes next() {
      Modes[] values = Modes.values();
      int nextOrdinal = (this.ordinal() + 1) % values.length;
      return values[nextOrdinal];
    }
  }

  Modes mode = Modes.POSITION;
  boolean lastStart = false;
  double setP = 0;

  void testCases() {
    double value;
    // Hiting the start button moves to the next control method
    boolean start = controller.start().getAsBoolean();
    if (start && !lastStart) {
      setSpeed(0);
      motor.setPosition(0);
      mode = mode.next(); // Get the next mode
      logf("***** Mode:%s for %s\n", mode, name);
    }
    lastStart = start;
    switch (mode) {
      case POSITION:
        value = controller.getHID().getPOV() / 10.0;
        if (value >= 0.0) {
          if (setP != value)
            logf("%s set position:%.2f\n", name, value);
          setPos(value);
          setP = value;
        }
        break;
      case VELOCITY:
        // POV 270 degrees is 100
        value = controller.getHID().getPOV() / (270.0 / 100.0);
        if (value >= 0) {
          if (setP != value)
            logf("%s set velocity:%.2f\n", name, value);
          setP = value;
          setVelocity(value);

        }
        break;
      case MOTIONMAGIC:
        value = controller.getHID().getPOV() / 2.0;
        if (value >= 0) {
          if (setP != value)
            logf("%s set magic motion position:%.2f\n", name, value);
          setP = value;
          setPositionMotionMagic(value);
        }
        break;
      case SPEED:
        value = Util.getSpeedFromTriggers(controller);
        if (Math.abs(value) > 0.05) {
          if (setP != value)
            logf("Set Test speed:%.2f\n", value);
          setP = value;
        }
        setSpeed(value);
        break;
    }
    // RobotContainer.setLedsForTestMode(mode.ordinal(), Modes.values().length);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> setSpeed(0.0))
        .withTimeout(0.5)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> setSpeed(0.0))
        .withTimeout(0.5)
        .andThen(sysId.dynamic(direction).withTimeout(1));
  }
}