// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// https://docs.wpilib.org/es/latest/docs/yearly-overview/yearly-changelog.html
//

package frc.robot;

import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;
import static frc.robot.utilities.Util.splashScreen;

import java.util.Optional;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.YawProvider;

// import dev.doglog.DogLog;
// import dev.doglog.DogLogOptions;

/**
 * file:///C:/Users/Public/wpilib/2025/documentation/rtd/frc-docs-latest/index.html#document-docs/software/support/support-resources
 *
 * <p>
 * software for 2025 --
 * https://docs.wpilib.org/en/latest/docs/zero-to-robot/step-2/wpilib-setup.html
 *
 * <p>
 * Sometimes the garbage collector won't run frequently enough to keep up with
 * the quantity of
 * allocations. As Java provides a way to trigger a garbage collection to occur,
 * running it on a
 * periodic basis may reduce peak memory usage. This can be done by adding a
 * Timer and a periodic
 * check:
 *
 * <p>
 * Timer m_gcTimer = new Timer(); public Robot() { m_gcTimer.start(); } public
 * void periodic() {
 * // run the garbage collector every 5 seconds if
 * (m_gcTimer.advanceIfElapsed(5)) { System.gc(); }
 * }
 */
public class Robot extends LoggedRobot {
  public static int count = 0;
  public static RobotContainer robotContainer;
  public static Optional<Alliance> alliance;
  public static boolean debug = true;
  public static Config config = new Config();
  public static double yaw;
  public static YawProvider yawProvider = new YawProvider();

  @Override
  public void robotInit() {
    config.getRobotTypeFromFile();
    Logger.addDataReceiver(new WPILOGWriter());
    Logger.addDataReceiver(new NT4Publisher());
    // Initialize URCL
    Logger.registerURCL(URCL.startExternal());

    // Initialize logging data from PDP
    // LoggedPowerDistribution.getInstance(Constants.powerDistributionCanId,
    // ModuleType.kRev);

    // Start AdvantageKit logger
    Logger.start();

    alliance = DriverStation.getAlliance();
    yawProvider.zeroYaw();
    splashScreen("1.5");
    robotContainer = new RobotContainer();

    var robotPlatform = robotContainer.robot();
    if (robotPlatform.isPresent()) {
    robotPlatform.get().robotInit();
    }
  }

  @Override
  public void teleopInit() {
    logf("Start Teleop\n");
    System.gc();

    var robotPlatform = robotContainer.robot();
    if (robotPlatform.isPresent()) {
    robotPlatform.get().teleopInit();
    }
  }

  @Override
  public void testInit() {
    // var robotPlatform = robotContainer.robot();
    // if (robotPlatform.isPresent()) {
    // robotPlatform.get().testInit();
    // }
  }

  @Override
  public void autonomousInit() {
    // var robotPlatform = robotContainer.robot();
    // if (robotPlatform.isPresent()) {
    // robotPlatform.get().testInit();
    // }
  }

  @Override
  public void robotPeriodic() {
    // var robotPlatform = robotContainer.robot();
    // if (robotPlatform.isPresent()) {
    //   robotPlatform.get().robotPeriodic();
    // }
    CommandScheduler.getInstance().run();
    if (count % 20 == 4) { // Update Dashboard every 20 cycles or 200 milliseconds (20 ms * 10)
      yaw = yawProvider.getYaw();
      SmartDashboard.putNumber("Yaw", round2(yaw));
    }
    count++;
  }

  @Override
  public void teleopPeriodic() {
    if (count % 100 == 0) {
      long mem = Runtime.getRuntime().freeMemory();
      SmartDashboard.putNumber("Free Mem", mem);
    }
    if (count % 50 == 0) {
      // System.gc();
    }

    var robotPlatform = robotContainer.robot();
    if (robotPlatform.isPresent()) {
      try {
        robotPlatform.get().teleopPeriodic();
      } catch (Exception e) {
      }
    }
  }

  @Override
  public void autonomousPeriodic() {
    // var robotPlatform = robotContainer.robot();
    // if (robotPlatform.isPresent()) {
    // robotPlatform.get().autonomousPeriodic();
    // }
  }

  @Override
  public void disabledPeriodic() {
    // var robotPlatform = robotContainer.robot();
    // if (robotPlatform.isPresent()) {
    // robotPlatform.get().disabledPeriodic();
    // }
  }

  @Override
  public void disabledInit() {
    // var robotPlatform = robotContainer.robot();
    // if (robotPlatform.isPresent()) {
    // robotPlatform.get().disabledInit();
    // }
  }

  @Override
  public void autonomousExit() {
    // var robotPlatform = robotContainer.robot();
    // if (robotPlatform.isPresent()) {
    // robotPlatform.get().autonomousExit();
    // }
  }

  @Override
  public void disabledExit() {
    // var robotPlatform = robotContainer.robot();
    // if (robotPlatform.isPresent()) {
    // robotPlatform.get().disabledExit();
    // }
  }

  // @Override
  // public void simulationPeriodic() {
  // if (count % 50 == 0) {
  // // logf("Count:%d\n", count);
  // robotContainer.testLeds();
  // }
  // }\

  private long lastMem = 0;
  private double lastTime = 0;

  private void logGCollections() {
    if (count % 20 == 0) {
      long mem = Runtime.getRuntime().freeMemory();
      if (mem > lastMem) {
        double ti = System.currentTimeMillis() / 1000.0;
        logf("A GC was done and freed:%d mem:%d time:%.2f", mem - lastMem, mem, ti - lastTime);
        lastTime = ti;

      }
      SmartDashboard.putNumber("Free Mem", mem);
      lastMem = mem;
    }
  }
}
