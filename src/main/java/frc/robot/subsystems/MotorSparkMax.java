package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Robot.robotContainer;
import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import com.revrobotics.spark.SparkRelativeEncoder;

/**
 * SPARK MAX controllers are initialized over CAN by constructing a SparkMax
 * object
 *
 * Look to:
 * 
 * https://docs.revrobotics.com/brushless/revlib/revlib-overview/migrating-to-revlib-2025
 * https://codedocs.revrobotics.com/java/com/revrobotics/spark/config/package-summary
 * https://www.chiefdelphi.com/t/rev-robotics-2024-2025/471083/139
 * 
 * for 2025 information
 *
 */

public class MotorSparkMax extends SubsystemBase {
    private SparkMax motor;
    private SparkBase motorS;
    private SparkRelativeEncoder encoder;
    private SparkMax followMotor;
    private String name;
    private int followId;
    private boolean brakeMode;
    private double lastSpeed;
    private SparkLimitSwitch forwardSwitch;
    private SparkLimitSwitch reverseSwitch;
    private boolean myLogging = true;
    private double lastDesiredPosition;
    private RelativeEncoder relEncoder;
    private double lastPos = 0;
    private SparkMaxConfig motorConfig;
    private double positionConversionFactor = 1.0;
    private double velocityConversionFactor = 1.0;
    private boolean testMode = false;
    private int numberCyclesForDisplay = 1000000;
    private final SysIdRoutine sysId;
    private MotorKrakenInputsAutoLogged inputs = new MotorKrakenInputsAutoLogged();
    // private final VoltageOut voltage = new VoltageOut(0);

    public MotorSparkMax(String name, int id, int followId, boolean brakeMode, boolean invert) {
        this.name = name;
        this.followId = followId;
        this.brakeMode = brakeMode;
        myMotorSpark(name, id, followId, false);

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        Time.ofBaseUnits(3.5, Seconds),
                        (state) -> Logger.recordOutput(name + "/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage), null, this));
    }

    void myMotorSpark(String name, int id, int followId, boolean logging) {
        this.name = name;
        this.followId = followId;
        myLogging = logging;
        logf("Start Spark Max %s id:%d\n", name, id);
        motor = new SparkMax(id, MotorType.kBrushless);
        {

        }
        ;
        setConfig(motor);
        if (followId > 0) {
            followMotor = new SparkMax(followId, MotorType.kBrushless);
            setConfig(followMotor);
        }
        setBrakeMode(brakeMode);
        relEncoder = motor.getEncoder();
        relEncoder.setPosition(0.0);

        if (followId > 0)
            logf("Created %s dual motors ids:<%d,%d> firmware:<%s,%s> \n", name, id,
                    followId, motor.getFirmwareString(), followMotor.getFirmwareString());
        else
            logf("Created %s motor id:%d firmware:%s \n", name, id,
                    motor.getFirmwareString());
    }

    public void setTestMode(boolean value) {
        testMode = value;
    }

    public void setLogging(boolean value) {
        myLogging = value;
    }

    private void setConfig(SparkMax motor) {
        /*
         * Create a new SPARK MAX configuration object. This will store the
         * configuration parameters for the SPARK MAX that we will set below.
         */
        motorConfig = new SparkMaxConfig();

        motorConfig.limitSwitch.forwardLimitSwitchEnabled(true);
        motorConfig.limitSwitch.reverseLimitSwitchEnabled(true);

        /*
         * Configure the encoder. For this specific example, we are using the
         * integrated encoder of the NEO, and we don't need to configure it. If
         * needed, we can adjust values like the position or velocity conversion
         * factors.
         */
        motorConfig.encoder
                .positionConversionFactor(positionConversionFactor)
                .velocityConversionFactor(velocityConversionFactor);

        // Configure the closed loop controller. We want to make sure we set the
        // feedback sensor as the primary encoder.
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control in slot 0
                .p(0.2, ClosedLoopSlot.kSlot0)
                .i(0, ClosedLoopSlot.kSlot0)
                .d(2, ClosedLoopSlot.kSlot0) // was 1
                .outputRange(-1, 1, ClosedLoopSlot.kSlot0);

        motorConfig.closedLoop
                // Set PID values for velocity control in slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1) // Note 5767 is max speed for Velocity PIDs
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for motion position control in slot 2
                .p(1, ClosedLoopSlot.kSlot2)
                .i(0, ClosedLoopSlot.kSlot2)
                .d(0, ClosedLoopSlot.kSlot2) // was 1
                .outputRange(-1, 1, ClosedLoopSlot.kSlot2);

        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for motion velocity control in slot 3
                .p(0.0002, ClosedLoopSlot.kSlot3)
                .i(0, ClosedLoopSlot.kSlot3)
                .d(0, ClosedLoopSlot.kSlot3) // was 1
                .outputRange(-1, 1, ClosedLoopSlot.kSlot3);

        motorConfig.closedLoop.maxMotion
                // Set MAXMotion parameters for velocity control in slot 1
                .maxAcceleration(3000, ClosedLoopSlot.kSlot1) // Was 500
                .maxVelocity(6000, ClosedLoopSlot.kSlot1)
                .allowedClosedLoopError(.1, ClosedLoopSlot.kSlot1)
                // Set MAXMotion parameters for position control in slot 2
                .maxAcceleration(3000, ClosedLoopSlot.kSlot2) // Was 500
                .maxVelocity(6000, ClosedLoopSlot.kSlot2)
                .allowedClosedLoopError(.1, ClosedLoopSlot.kSlot2)
                // Set MAXMotion parameters for velocity control in slot 3
                .maxAcceleration(3000, ClosedLoopSlot.kSlot3) // Was 500
                .maxVelocity(6000, ClosedLoopSlot.kSlot3)
                .allowedClosedLoopError(.1, ClosedLoopSlot.kSlot3);

        // Apply the configuration to the SPARK MAX.
        // kPersistParameters is used to ensure the configuration is not lost when
        // the SPARK MAX loses power. This is useful for power cycles that may occur
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // config.closedLoop.pidf(motorC.kP, motorC.kI, motorC.kD, motorC.kFF);
    // config.closedLoop.maxOutput(motorC.maxOutput);
    // config.closedLoop.smartMotion.maxVelocity(motorC.maxVelocity);
    // config.closedLoop.smartMotion.maxAcceleration(motorC.maxAcceration);
    // config.closedLoop.maxMotion.maxVelocity(motorC.maxVelocity);
    // config.closedLoop.maxMotion.maxAcceleration(motorC.maxAcceration);
    // config.smartCurrentLimit(motorC.currentLimit);
    // config.openLoopRampRate(motorC.openLoopRampRate);
    // config.closedLoopRampRate(motorC.openLoopRampRate);

    public void setBrakeMode(boolean value) {
        motorConfig.idleMode(value ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public double getSpeed() {
        return relEncoder.getVelocity();
    }

    public double getError() {
        return Math.abs(lastDesiredPosition - getPos());
    }

    public void setSpeed(double speed) {
        if (lastSpeed == speed)
            return;
        motor.set(speed);
        lastSpeed = speed;
    }

    public void setPos(double position) {
        lastDesiredPosition = position;
        motor.getClosedLoopController().setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void setVelocity(double velocity) {
        motor.getClosedLoopController().setReference(velocity, SparkFlex.ControlType.kVelocity,
                ClosedLoopSlot.kSlot1);
    }

    public void setPosMotionMagic(double position) {
        lastDesiredPosition = position;
        motor.getClosedLoopController().setReference(position, ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot2);
    }

    public void setVelocityMotionMagic(double velocity) {
        motor.getClosedLoopController().setReference(velocity, SparkFlex.ControlType.kMAXMotionVelocityControl,
                ClosedLoopSlot.kSlot3);
    }

    public void stopMotor() {
        motor.stopMotor();
        lastSpeed = 0;
    }

    public double getMotorVoltage() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    public double getMotorCurrent() {
        return motor.getOutputCurrent();
    }

    public double getFollowCurrent() {
        if (followMotor == null) {
            return 0;
        }
        return followMotor.getOutputCurrent();
    }

    public double getPos() {
        return relEncoder.getPosition();
    }

    public void zeroEncoder() {
        relEncoder.setPosition(0);
    }

    public void setEncoderPosition(double position) {
        relEncoder.setPosition(position);
    }

    public boolean getForwardLimitSwitch() {
        return forwardSwitch.isPressed();
    }

    public boolean getReverseLimitSwitch() {
        return reverseSwitch.isPressed();
    }

    public void logPeriodic() {
        double pos = getPos();
        if (pos != lastPos) {
            lastPos = pos;
            if (myLogging) {
                logf("%s\n", getMotorVCS(motor));
                if (followId > 0) {
                    logf("%s\n", getMotorVCS(followMotor));
                }
            }
        }
    }

    public void periodic() {
        if (Robot.count % numberCyclesForDisplay == 0) {
            SmartDashboard.putNumber("Pos", getPos());
            SmartDashboard.putNumber("Cur", round2(motor.getOutputCurrent()));
            SmartDashboard.putNumber("Vel", round2(getSpeed()));
            SmartDashboard.putString("Mode", mode.toString());
            SmartDashboard.putNumber("RPM", round2(getSpeed() / velocityConversionFactor / 60));
            SmartDashboard.putNumber("Err", getError());
        }
        if (Robot.count % 10 == 0) {
            logPeriodic();
        }
        if (testMode) {
            testCases();
        }

        inputs.position = Rotations.of(getPos());
        inputs.velocity = RPM.of(getSpeed());
        inputs.appliedVolts = Volts.of(getMotorVoltage());
        inputs.currentStatorAmps = Amps.of(getMotorCurrent());
        inputs.currentSupplyAmps = Amps.of(getMotorCurrent());
        Logger.processInputs(name, inputs);

    }

    void testTimes() {
        long startTime = RobotController.getFPGATime();
        double sp = motor.get();
        long endTime1 = RobotController.getFPGATime();
        double cur = motor.getOutputCurrent();
        long endTime2 = RobotController.getFPGATime();
        double enc = relEncoder.getPosition();
        long endTime3 = RobotController.getFPGATime();
        logf("%s motor sp:%.2f pos:%.0f\n", name, sp, cur, enc);
        long endTime = RobotController.getFPGATime();
        System.out.printf("get:%d cur:%d enc:%d log:%d total:%d\n", endTime1 - startTime, endTime2 - endTime1,
                endTime3 - endTime2, endTime - endTime3, endTime - startTime);
    }

    private String getMotorVCS(SparkMax motor) {
        double bussVoltage = motor.getBusVoltage();
        double outputCurrent = motor.getOutputCurrent();
        return String.format("%s motor volts:%.2f cur:%.2f velocity:%.2f power:%.2f sp:%.3f", name, bussVoltage,
                outputCurrent, motor.getAbsoluteEncoder().getVelocity(), bussVoltage * outputCurrent, lastSpeed);
    }

    public double getLastSpeed() {
        return motor.getAbsoluteEncoder().getVelocity();
    }

    public void logMotorVCS() {
        if (Math.abs(getLastSpeed()) > .01) {
            logf("%s\n", getMotorVCS(motor));
            if (followId > 0)
                logf("%s\n", getMotorVCS(followMotor));
        }
    }

    public void setSmartTicks(int numberLoopsForDisplay) {
        if (numberLoopsForDisplay <= 0)
            this.numberCyclesForDisplay = Integer.MAX_VALUE;
        else
            this.numberCyclesForDisplay = numberLoopsForDisplay;
    }

    private void setVoltage(Voltage value) {
        /* Use velocity voltage */
        motor.setVoltage(value);
    }

    enum Modes {
        POSITION, VELOCITY, POSMOTIONMAGIC, VELMOTIONMAGIC, SPEED;

        public Modes next() {
            Modes[] values = Modes.values();
            int nextOrdinal = (this.ordinal() + 1) % values.length;
            return values[nextOrdinal];
        }
    }

    Modes mode = Modes.POSITION;
    boolean lastStart = false;
    double setPoint = 0;

    void testCases() {
        CommandXboxController driveController = RobotContainer.driveController;
        double value;
        // Hiting the start button moves to the next control method
        boolean start = driveController.start().getAsBoolean();
        if (start && !lastStart) {
            stopMotor();
            setEncoderPosition(0.0);
            mode = mode.next(); // Get the next mode
            logf("***** Mode:%s for %s\n", mode, name);
        }
        lastStart = start;
        switch (mode) {
            case POSITION:
                value = driveController.getHID().getPOV() / 10.0;
                if (value >= 0.0) {
                    if (setPoint != value)
                        logf("%s set position:%.6f\n", name, value);
                    setPos(value);
                    setPoint = value;
                }
                break;
            case VELOCITY:
                value = driveController.getHID().getPOV() * 20.0;
                if (value >= 0.0) {
                    if (setPoint != value)
                        logf("%s set velocity:%.2f\n", name, value);
                    setVelocity(value);
                    setPoint = value;
                }
                break;
            case POSMOTIONMAGIC:
                value = driveController.getHID().getPOV() / 10.0;
                if (value >= 0.0) {
                    if (setPoint != value)
                        logf("%s set position motion magic:%.6f\n", name, value);
                    setPosMotionMagic(value);
                    setPoint = value;
                }
                break;
            case VELMOTIONMAGIC:
                value = driveController.getHID().getPOV() * 20.0;
                if (value >= 0.0) {
                    setVelocityMotionMagic(value);
                    setPoint = value;
                    logf("%s set velocity motion magic:%.2f\n", name, value);
                }
                break;
            case SPEED:
                value = robotContainer.getSpeedFromTriggers();
                if (Math.abs(value) > 0.15) {
                    if (setPoint != value)
                        logf("%s Set speed:%.2f\n", name, value);
                } else {
                    value = 0.0;
                }
                setSpeed(value);
                setPoint = value;

                break;
        }
        RobotContainer.setLedsForTestMode(mode.ordinal(), Modes.values().length);
        SmartDashboard.putNumber("SetP", setPoint);
    }

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