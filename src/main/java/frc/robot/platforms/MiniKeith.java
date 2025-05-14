package frc.robot.platforms;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSRX;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.MotorFlex;
import frc.robot.subsystems.MotorKraken;
import frc.robot.subsystems.MotorSRX;
import frc.robot.subsystems.MotorSparkMax;

public class MiniKeith implements RobotRunnable {
    private final static int LED_COUNT = 30;

    private final static LedSubsystem m_leds = new LedSubsystem();
    private static DrivetrainSRX m_drivetrain;
    private static CommandXboxController m_driveHID;

    private final MotorFlex m_motorFlex = new MotorFlex("motorFlex", 10, -1, false);
    private final MotorSparkMax m_motorSpark = new MotorSparkMax("sparkMax", 11, -1, false, false);
    private final MotorKraken m_motorKraken = new MotorKraken("motorKraken", 16, -1, true);
    private final MotorSRX m_motorSRX = new MotorSRX("motorSRX", 14, 0, true);

    // Code to display CANCoder value
    private final CANcoder m_canCoder = new CANcoder(20);

    private boolean testFlex = false;
    private boolean testSmartMax = false;
    private boolean testKraken = false;
    private boolean testSRX = false;

    enum Motors {
        FLEX, MAX, KRAKEN, SRX;

        public Motors next() {
            Motors[] values = Motors.values();
            int nextOrdinal = (this.ordinal() + 1) % values.length;
            return values[nextOrdinal];
        }
    }

    private Motors motors = Motors.FLEX; // Set default motor for testing

    public MiniKeith(CommandXboxController driveHID) {
        m_driveHID = driveHID;
        m_drivetrain = new DrivetrainSRX(driveHID.getHID());

        m_motorSRX.setupForTestCasesRedMotor();
        setMotorForTest();

        Commands.run(
                () -> SmartDashboard.putNumber("CanCo", m_canCoder.getPosition().getValueAsDouble()))
                .ignoringDisable(true).schedule();

        // Code to have leds reflect value of LeftX
        Commands.run(
                () -> setLedsLeftX()).ignoringDisable(true).schedule();

        m_driveHID.back().onTrue(
                new InstantCommand(new Runnable() {
                    public void run() {
                        setMotorForTest();
                    }
                }));

        m_driveHID.back().whileTrue(
                new InstantCommand(
                        new Runnable() {
                            public void run() {
                                Robot.yawProvider.zeroYaw();
                                logf("Hit back on Game Pad\n");
                            }
                        }));
    }

    public void setLedsLeftX() {
        int num = LED_COUNT - 6;
        double value = m_driveHID.getLeftX();
        if (value < 0.0)
            value = 0.0;
        m_leds.setRangeOfColor(6, (int) (value * num), num, 0, 127, 0);
    }

    public static void setLedsForTestMode(int index, int number) {
        m_leds.setRangeOfColor(0, number, 0, 0, 0);
        m_leds.setRangeOfColor(0, index, 0, 50, 0);
    }

    private void setMotorForTest() {
        testFlex = false;
        testSmartMax = false;
        testKraken = false;
        testSRX = false;
        motors = motors.next(); // Get the next mode
        logf("************** Motor:%s\n", motors.toString());
        switch (motors) {
            case FLEX:
                testFlex = true;
                break;
            case MAX:
                testSmartMax = true;
                break;
            case KRAKEN:
                testKraken = true;
                break;
            case SRX:
                testSRX = true;
                break;
        }
        m_motorFlex.setTestMode(testFlex);
        m_motorFlex.setLogging(testFlex);
        m_motorFlex.setSmartTicks(testFlex ? 2 : 0);
        m_motorSpark.setTestMode(testSmartMax);
        m_motorSpark.setLogging(testSmartMax);
        m_motorSpark.setSmartTicks(testSmartMax ? 2 : 0);
        m_motorKraken.setTestMode(testKraken);
        m_motorKraken.setLogging(testKraken);
        m_motorKraken.setSmartTicks(testKraken ? 1 : 0);
        m_motorSRX.setTestMode(testSRX);
        m_motorSRX.setLogging(testSRX);
        m_motorSRX.setSmartTicks(testSRX ? 2 : 0);
        SmartDashboard.putString("Motor", motors.toString());

        if (m_motorKraken != null && testKraken) {
            m_driveHID.a().whileTrue(m_motorKraken.sysIdDynamic(Direction.kForward));
            m_driveHID.b().whileTrue(m_motorKraken.sysIdDynamic(Direction.kReverse));
            m_driveHID.x().whileTrue(m_motorKraken.sysIdQuasistatic(Direction.kForward));
            m_driveHID.y().whileTrue(m_motorKraken.sysIdQuasistatic(Direction.kReverse));
        }
        if (m_motorSpark != null) {
            m_driveHID.a().whileTrue(m_motorSpark.sysIdDynamic(Direction.kForward));
            m_driveHID.b().whileTrue(m_motorSpark.sysIdDynamic(Direction.kReverse));
            m_driveHID.x().whileTrue(m_motorSpark.sysIdQuasistatic(Direction.kForward));
            m_driveHID.y().whileTrue(m_motorSpark.sysIdQuasistatic(Direction.kReverse));
        }
    }

}
