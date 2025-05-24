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
import frc.robot.subsystems.MotorTester;

public class MiniKeith implements RobotRunnable {
    private final static int LED_COUNT = 30;

    private final static LedSubsystem m_leds = new LedSubsystem();
    private static DrivetrainSRX m_drivetrain;
    private static CommandXboxController m_driveHID;

    private MotorFlex m_motorFlex;
    private MotorSparkMax m_motorSpark;
    private MotorKraken m_motorKraken;
    private MotorSRX m_motorSRX;

    // Code to display CANCoder value
    private final CANcoder m_canCoder = new CANcoder(20);

    private MotorTester.Motors m_testedMotor = MotorTester.Motors.FLEX; // Set default motor for testing
    private MotorTester m_motorTester;

    public MiniKeith(CommandXboxController driveHID) {
        m_motorFlex = new MotorFlex("motorFlex", 10, -1, driveHID, false);
        m_motorSpark = new MotorSparkMax("sparkMax", 11, -1, driveHID, false, false);
        m_motorKraken = new MotorKraken("motorKraken", 16, -1, driveHID, true);
        m_motorSRX = new MotorSRX("motorSRX", 14, 0, driveHID, true);
        m_driveHID = driveHID;
        m_drivetrain = new DrivetrainSRX(driveHID.getHID());
        m_motorTester = new MotorTester(m_motorFlex, m_motorSpark, m_motorKraken, m_motorSRX);

        m_motorSRX.setupForTestCasesRedMotor();

        Commands.run(() -> SmartDashboard.putNumber("CanCo", m_canCoder.getPosition().getValueAsDouble()))
                .ignoringDisable(true).schedule();

        // Code to have leds reflect value of LeftX
        Commands.run(() -> setLedsLeftX()).ignoringDisable(true).schedule();

        m_driveHID.back().onTrue(new InstantCommand(new Runnable() {
            public void run() {
                try {
                    m_testedMotor = m_motorTester.selectNextMotor();
                } catch (Exception e) {
                    logf("Unable to start MotorTester: %s", e.toString());
                }
                switch (m_testedMotor) {
                    case FLEX:
                        break;
                    case MAX:
                        m_driveHID.a().whileTrue(m_motorSpark.sysIdDynamic(Direction.kForward));
                        m_driveHID.b().whileTrue(m_motorSpark.sysIdDynamic(Direction.kReverse));
                        m_driveHID.x().whileTrue(m_motorSpark.sysIdQuasistatic(Direction.kForward));
                        m_driveHID.y().whileTrue(m_motorSpark.sysIdQuasistatic(Direction.kReverse));
                        break;
                    case KRAKEN:
                        m_driveHID.a().whileTrue(m_motorKraken.sysIdDynamic(Direction.kForward));
                        m_driveHID.b().whileTrue(m_motorKraken.sysIdDynamic(Direction.kReverse));
                        m_driveHID.x().whileTrue(m_motorKraken.sysIdQuasistatic(Direction.kForward));
                        m_driveHID.y().whileTrue(m_motorKraken.sysIdQuasistatic(Direction.kReverse));
                        break;
                    case SRX:
                        break;
                    default:
                        break;
                }
            }
        }));

        m_driveHID.back().whileTrue(new InstantCommand(new Runnable() {
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
}
