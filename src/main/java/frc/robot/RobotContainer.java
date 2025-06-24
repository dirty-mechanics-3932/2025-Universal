package frc.robot;

// import static frc.robot.Robot.yaw;
import static frc.robot.utilities.Util.logf;

import java.util.Optional;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.platforms.BlondeMini;
import frc.robot.platforms.DarrylMini;
import frc.robot.platforms.Mando;
import frc.robot.platforms.MiniIsaac;
import frc.robot.platforms.KeithMini;
import frc.robot.platforms.MiniMini;
import frc.robot.platforms.ParadeSrxDriveRobots;
import frc.robot.platforms.RobotRunnable;
import frc.robot.platforms.Sibling2025;
import frc.robot.subsystems.DrivetrainJaguar;
import frc.robot.subsystems.DrivetrainSRX;

/**
 * This class is where the bulk of the robot should be declared. Since be
 * declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls). Instead,
 * the structure ofthe
 * robot (including subsystems, commands, and button mappings) should be
 * declared here.
 */
public class RobotContainer {
  private Optional<RobotRunnable> runnableRobot = Optional.empty();

  //public static LedSubsystem leds = new LedSubsystem();
  public DrivetrainSRX drivetrainSRX;

  // private MotorFlex motorFlex;
  // private MotorSparkMax motorSparkMax;
  // private MotorKraken motorKraken;
  // private MotorSRX motorSRX;

  // enum Motors {
  // FLEX, MAX, KRAKEN, SRX;

  // public Motors next() {
  // Motors[] values = Motors.values();
  // int nextOrdinal = (this.ordinal() + 1) % values.length;
  // return values[nextOrdinal];
  // }
  // }

  // private Motors motors = Motors.FLEX; // Set default motor for testing

  // private void setMotorForTest() {
  // testFlex = false;
  // testSmartMax = false;
  // testKraken = false;
  // testSRX = false;
  // motors = motors.next(); // Get the next mode
  // logf("************** Motor:%s\n", motors.toString());
  // switch (motors) {
  // case FLEX:
  // testFlex = true;
  // break;
  // case MAX:
  // testSmartMax = true;
  // break;
  // case KRAKEN:
  // testKraken = true;
  // break;
  // case SRX:
  // testSRX = true;
  // break;
  // }
  // motorFlex.setTestMode(testFlex);
  // motorFlex.setLogging(testFlex);
  // motorFlex.setSmartTicks(testFlex ? 2 : 0);
  // motorSparkMax.setTestMode(testSmartMax);
  // motorSparkMax.setLogging(testSmartMax);
  // motorSparkMax.setSmartTicks(testSmartMax ? 2 : 0);
  // motorKraken.setTestMode(testKraken);
  // motorKraken.setLogging(testKraken);
  // motorKraken.setSmartTicks(testKraken ? 1 : 0);
  // motorSRX.setTestMode(testSRX);
  // motorSRX.setLogging(testSRX);
  // motorSRX.setSmartTicks(testSRX ? 2 : 0);
  // SmartDashboard.putString("Motor", motors.toString());
  // }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CommandXboxController driveController = new CommandXboxController(Config.DRIVE_CONTROLLER_PORT);
    XboxController driveHID = driveController.getHID();

    // double kP;
    // double kI;
    // double kD;
    // PIDController pidNeo = new PIDController(kP, kI, kD);

    // Set the default Robot Mode to Cube
    switch (Config.robotType) {
      case Simulation:
        driveController.back().whileTrue(zeroYawCommand);
        break;
      case BlondeMini:
        runnableRobot = Optional.of(new BlondeMini(driveController));
        break;
      case DarrylMini:
        runnableRobot = Optional.of(new DarrylMini());
        break;
      case MiniMini:
        runnableRobot = Optional.of(new MiniMini(driveController));
        break;
      case KeithMini: // Test mini
        runnableRobot = Optional.of(new KeithMini(driveController));
        break;
      case Squidward:
        runnableRobot = Optional.of(new ParadeSrxDriveRobots(driveHID, "Squidward"));
        break;
      case Kevin: // Ginger Bread Robot
        runnableRobot = Optional.of(new ParadeSrxDriveRobots(driveHID, "Kevin"));
        break;
      case Wooly: // Big ball shooter
        // Uses Jaguars for drive train and shooter
        // Uses PCM to control shooter tilt and shooter activate
        Robot.config.driveTrainJaguar = true;
        Robot.config.enableCompressor = true;
        Robot.config.pneumaticType = PneumaticsModuleType.CTREPCM;
        new DrivetrainJaguar(driveHID);
        break;
      case Mando: // Train engine
        runnableRobot = Optional.of(new Mando(driveHID));
        break;
      case Sibling2025:
        runnableRobot = Optional.of(new Sibling2025(driveHID));
        break;
      case MiniIsaac:
        runnableRobot = Optional.of(new MiniIsaac());
        break;
    }
    logf("Finished Creating RobotContainer\n");
  }

  public Optional<RobotRunnable> robot() {
    return runnableRobot;
  }

  // The following code is an attempt to learn how to program commands

  // Command hit = new InstantCommand(
  // new Runnable() {
  // public void run() {
  // logf("Hit Button\n");
  // }
  // });

  // public double squareWithSign(double v) {
  // return (v < 0) ? -(v * v) : (v * v);
  // }

  // // Create a command to execute when the DIO switch is hit
  // Command testTrigger = new InstantCommand(
  // new Runnable() {
  // public void run() {
  // logf("DIO Switch Hit\n");
  // }
  // });

  // // From subsystem testTriggers run the getSwitch method if true
  // TestTriggers triggers = new TestTriggers();
  // Trigger tr = new Trigger(triggers::getSwitch).onTrue(testTrigger);

  // class hitButton extends Command {
  // @Override
  // public void execute() {
  // // Your code to run when the button is pressed, such as moving a motor
  // System.out.println("My command is running!");
  // }
  // }

  // Command leftxToLeds = new InstantCommand(
  // new Runnable() {
  // public void run() {
  // int num = Config.numberOfLeds - 6;
  // double value = RobotContainer.driveController.getLeftX();
  // if (value < 0.0)
  // value = 0.0;
  // leds.setRangeOfColor(6, (int) (value * num), num, 0, 127, 0);
  // }
  // });

  Command zeroYawCommand = new InstantCommand(
      new Runnable() {
        public void run() {
          Robot.yawProvider.zeroYaw();
        }
      });

  // Trigger tr = new Trigger(triggers::getSwitch);

  // Initializes a DigitalInput
  // DigitalInput input = new DigitalInput(Robot.config.DIOTestTrigger);
  // Creates a Debouncer in "both" mode.
  // Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

  // void deB() {
  // // If false the signal must go true for at least .1 seconds before read
  // if (m_debouncer.calculate(input.get())) {
  // logf("Input Changed:%b\n", input.get());
  // }
  // }
}
