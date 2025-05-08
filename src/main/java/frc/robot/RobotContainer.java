package frc.robot;

// import static frc.robot.Robot.yaw;
import static frc.robot.utilities.Util.logf;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Config.RobotType;
import frc.robot.platforms.MiniMini;
import frc.robot.platforms.RobotRunnable;
import frc.robot.subsystems.DrivetrainJaguar;
import frc.robot.subsystems.DrivetrainSRX;
import frc.robot.subsystems.DrivetrainSpark;
import frc.robot.subsystems.DrivetrainTestSwerve;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.MotorFlex;
import frc.robot.subsystems.MotorKraken;
import frc.robot.subsystems.MotorSRX;
import frc.robot.subsystems.MotorSparkMax;
import frc.robot.subsystems.PID;
import frc.robot.subsystems.TestTriggers;

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
  private Optional<RobotRunnable> runnableRobot;
  public static final CommandXboxController driveController = new CommandXboxController(2);
  private static final XboxController driveHID = driveController.getHID();

  public static LedSubsystem leds = new LedSubsystem();
  public DrivetrainSRX drivetrainSRX;

  private TestTriggers triggers = new TestTriggers();
  private CANcoder canCoder;

  private boolean testFlex = false;
  private boolean testSmartMax = true;
  private boolean testKraken = false;
  private boolean testSRX = false;
  private MotorFlex motorFlex;
  private MotorSparkMax motorSparkMax;
  private MotorKraken motorKraken;
  private MotorSRX motorSRX;

  enum Motors {
    FLEX, MAX, KRAKEN, SRX;

    public Motors next() {
      Motors[] values = Motors.values();
      int nextOrdinal = (this.ordinal() + 1) % values.length;
      return values[nextOrdinal];
    }
  }

  private Motors motors = Motors.FLEX; // Set default motor for testing

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
    motorFlex.setTestMode(testFlex);
    motorFlex.setLogging(testFlex);
    motorFlex.setSmartTicks(testFlex ? 2 : 0);
    motorSparkMax.setTestMode(testSmartMax);
    motorSparkMax.setLogging(testSmartMax);
    motorSparkMax.setSmartTicks(testSmartMax ? 2 : 0);
    motorKraken.setTestMode(testKraken);
    motorKraken.setLogging(testKraken);
    motorKraken.setSmartTicks(testKraken ? 1 : 0);
    motorSRX.setTestMode(testSRX);
    motorSRX.setLogging(testSRX);
    motorSRX.setSmartTicks(testSRX ? 2 : 0);
    SmartDashboard.putString("Motor", motors.toString());
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set the default Robot Mode to Cube
    switch (Config.robotType) {
      case Simulation:
        break;
      case BlondeMini:
        // new DrivetrainSRX(driveHID);
        motorKraken = new MotorKraken("testSysid", 25, -1, true);
        boolean testSmartMaxBlonde = true;
        MotorSparkMax motor = new MotorSparkMax("TestMax", 20, -1, false, false);
        if (testSmartMaxBlonde) {
          motor.setLogging(true);
          motor.setTestMode(true);
        } else {
          Command blondeMove = Commands.run(() -> motor.setSpeed(getSpeedFromTriggers()), motor);
          blondeMove.ignoringDisable(true).schedule();
        }
        break;
      case DarrylMini:
        new DrivetrainSRX(driveHID);
        MotorSRX dmotor = new MotorSRX("DarrylSRX", 10, -1, true);
        Command darrylMoveBack = Commands.run(() -> dmotor.setSpeed(getSpeedFromTriggers()), dmotor);
        darrylMoveBack.ignoringDisable(true).schedule();
        break;
      case MiniMini:
        runnableRobot = Optional.of(new MiniMini(3, 10, driveController));

//         MotorSRX redMotor = new MotorSRX("RedMotor", 10, -1, true);
//         PID positionPID = new PID("Pos", .08, 0, 0, 0, 0, -1, 1, true);
//         PID velocityPID = new PID("Vel", .005, 0, 0, 0, 1.5, -1, 1, true);
//         // Motion Magic messes things up positionPID.setMotionMagicSRX(.5, 2.0);
//         redMotor.setPositionPID(positionPID, 0, FeedbackDevice.QuadEncoder); // set pid for SRX
//         redMotor.setVelocityPID(velocityPID, 1, FeedbackDevice.QuadEncoder);

//         MotorFlex flexMotor = new MotorFlex("FlexMotor", 3, -1, true);
//         flexMotor.setLogging(true);
//         flexMotor.setTestMode(true);
//         redMotor.setUpForTestCases(leds);
//         redMotor.setLogging(true);
//         redMotor.setEncoderTicksPerRev(2048);
//         Command redMoveCmd = Commands.run(() ->
//         redMotor.setSpeed(driveController.getLeftTriggerAxis()), redMotor);
//         Command neoMoveCmd = Commands.run(() ->
//         flexMotor.setSpeed(driveController.getRightTriggerAxis()), flexMotor);
//         new ScheduleCommand(Commands.parallel(redMoveCmd,
//         neoMoveCmd).ignoringDisable(true)).schedule();
//         Command miniMove = Commands.run(() ->
//         flexMotor.setSpeed(driveController.getLeftTriggerAxis()), flexMotor);
//         driveController.start().onTrue(miniMove);
//         new ScheduleCommand(miniMove);
        break;
      case MiniKeith: // Test mini
        // Use Talon SRX for drive train
        drivetrainSRX = new DrivetrainSRX(driveHID);
        // Setup to test Flex Motor
        motorFlex = new MotorFlex("Flex", 10, -1, false);
        motorSparkMax = new MotorSparkMax("SmartMax", 11, -1, false, false);
        motorKraken = new MotorKraken("Kraken", 16, -1, true);
        motorSRX = new MotorSRX("SRX", 14, 0, true);
        motorSRX.setupForTestCasesRedMotor();
        setMotorForTest();
       // Code to display CANCoder value
        canCoder = new CANcoder(20);
        Command miniCancoder = Commands.run(
            () -> SmartDashboard.putNumber("CanCo", canCoder.getPosition().getValueAsDouble()));
        miniCancoder.ignoringDisable(true).schedule();
         
        // Code to have leds reflect value of LeftX
        Command leftxToLeds = Commands.run(
          () -> setLedsLeftX());
        leftxToLeds.ignoringDisable(true).schedule();
        break;
      case Squidward:
        drivetrainSRX = new DrivetrainSRX(driveHID);
        // Uses Talon SRX for drive train())
        break;
      case Kevin: // Ginger Bread Robot
        // Uses Talon SRX for drive train
        drivetrainSRX = new DrivetrainSRX(driveHID);
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
        // Use SparkMax motors for drive train
        new DrivetrainSpark(driveHID);
        break;
      case Sibling2025:
        new DrivetrainTestSwerve(driveHID);
        break;
    }
    logf("Finished Creating RobotContainer\n");
    if (Config.robotType != RobotType.Simulation) {
      configureButtonBindings();
    }
    SmartDashboard.putData("UpdatePID", hit);
  }

  public static void setLedsForTestMode(int index, int number) {
    leds.setRangeOfColor(0, number, 0, 0, 0);
    leds.setRangeOfColor(0, index, 0, 50, 0);
  }

  public double getSpeedFromTriggers() {
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

  // Play with string encoder
  AnalogInput analog = new AnalogInput(3);

  public void setLedsForStringEncoder() {
    int v = (int) driveController.getHID().getPOV() / 4;
    leds.setOneLed(6, v, v, v);
    SmartDashboard.putNumber("Volts", analog.getVoltage());
    SmartDashboard.putNumber("Value", analog.getValue());
  }

  public void setLedsLeftX() {
      int num = Config.numberOfLeds - 6;
      double value = RobotContainer.driveController.getLeftX();
      if (value < 0.0)
        value = 0.0;
      leds.setRangeOfColor(6, (int) (value * num), num, 0, 127, 0);
  }

  // Command h = Commands.run(() -> logf("Hit\f"));

  Command hit = new InstantCommand(
      new Runnable() {
        public void run() {
          logf("Hit Button\n");
        }
      });

  public double squareWithSign(double v) {
    return (v < 0) ? -(v * v) : (v * v);
  }

  // The following code is an attempt to learn how to program commands

  // Create a command to execute when the DIO switch is hit
  Command testTrigger = new InstantCommand(
      new Runnable() {
        public void run() {
          logf("DIO Switch Hit\n");
        }
      });

  // From subsystem testTriggers run the getSwitch method if true
  Trigger tr = new Trigger(triggers::getSwitch).onTrue(testTrigger);

  class hitButton extends Command {
    @Override
    public void execute() {
      // Your code to run when the button is pressed, such as moving a motor
      System.out.println("My command is running!");
    }
  }

  // Command leftxToLeds = new InstantCommand(
  //     new Runnable() {
  //       public void run() {
  //         int num = Config.numberOfLeds - 6;
  //         double value = RobotContainer.driveController.getLeftX();
  //         if (value < 0.0)
  //           value = 0.0;
  //         leds.setRangeOfColor(6, (int) (value * num), num, 0, 127, 0);
  //       }
  //    });

  Command zeroYawCommand = new InstantCommand(
      new Runnable() {
        public void run() {
          Robot.yawProvider.zeroYaw();
        }
      });


  // Trigger tr = new Trigger(triggers::getSwitch);

  private void configureButtonBindings() {

    if (Config.robotType == RobotType.MiniKeith) {
      driveController.back().onTrue(
          new InstantCommand(
              new Runnable() {
                public void run() {
                  setMotorForTest();
                }
              }));
    }
    driveController.back()
        .whileTrue(
            new InstantCommand(
                new Runnable() {
                  public void run() {
                    Robot.yawProvider.zeroYaw();
                    logf("Hit back on Game Pad\n");
                  }
                }));

    if (motorKraken != null && testKraken) {
      driveController.a().whileTrue(motorKraken.sysIdDynamic(Direction.kForward));
      driveController.b().whileTrue(motorKraken.sysIdDynamic(Direction.kReverse));
      driveController.x().whileTrue(motorKraken.sysIdQuasistatic(Direction.kForward));
      driveController.y().whileTrue(motorKraken.sysIdQuasistatic(Direction.kReverse));
    }
  }

  // Initializes a DigitalInput
  DigitalInput input = new DigitalInput(Robot.config.DIOTestTrigger);
  // Creates a Debouncer in "both" mode.
  Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

  void deB() {
    // If false the signal must go true for at least .1 seconds before read
    if (m_debouncer.calculate(input.get())) {
      logf("Input Changed:%b\n", input.get());
    }
  }

  public Optional<RobotRunnable> robot() { return runnableRobot; }
}
