package frc.robot.utilities;

import frc.robot.Config;
import frc.robot.Robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Util {
  // Logging Methods
  // Use tail -f /home/lvuser/FRC_UserProgram.log | grep '++++\|----\|????' on
  // roboRio to see important commands

  public static void log(String s) {
    DateFormat dateFormat = new SimpleDateFormat("HH:mm:ss-SSS ");
    dateFormat.setTimeZone(TimeZone.getTimeZone("America/New_York"));
    Date date = new Date();
    System.out.println(dateFormat.format(date) + s);
  }

  public static void logf(String pattern, Object... arguments) {
    try {
      DateFormat dateFormat = new SimpleDateFormat("HH:mm:ss-SSS ");
      dateFormat.setTimeZone(TimeZone.getTimeZone("America/New_York"));
      System.out.printf((dateFormat.format(new Date()) + pattern), arguments);
    } catch (Exception e) {
      System.err.println("\nAn error occurred while logging! Pattern: " + pattern);
      e.printStackTrace();
    }
  }

  public static void loginfo(String pattern, Object... arguments) {
    if (Robot.debug)
      logf(pattern, arguments);
  }

  // Rounding Methods

  public static double round0(double d) {
    return Math.round(d);
  }

  public static double round1(double d) {
    return Math.round(d * 10D) / 10D;
  }

  public static double round2(double d) {
    return Math.round(d * 100D) / 100D;
  }

  public static double round3(double d) {
    return Math.round(d * 1000D) / 1000D;
  }

  public static double round4(double d) {
    return Math.round(d * 10000) / 10000.0;
  }

  // Take an angle and convert it to -180 to 180
  public static double normalizeAngle(double angle) {
    double a = (angle + 180) % 360;
    if (a < 0)
      a += 360;
    return a - 180;
  }

  // Take an angle and convert it to 0 to 360
  public static double unNormalilzeAngle(double angle) {
    double a = angle % 360;
    if (a < 0)
      a += 360;
    return a;
  }

  // Convert inches to meters
  public static double itm(double i) {
    return i * 2.54 / 100;
  }

  public static String showFileTime() {
    File file = new File("/home/lvuser/robotCommand");
    SimpleDateFormat sdf = new SimpleDateFormat("MM/dd/yyyy HH:mm:ss");
    return sdf.format(file.lastModified());
  }

  public static String getRobotType() {
    String line;
    File filePath = new File("/home/lvuser/deploy/robotType.txt");
    try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
      line = br.readLine();
    } catch (Exception e) {
      //e.printStackTrace();
      line = "Not Created";
    }
    return line;
  }

  public static void splashScreen(String version) {
    logf("**********************************************************************\n");
    logf("Robot Type %s Started compiled:%s version:%s\n",
        Config.robotType, Util.showFileTime(), version);
    logf("Alliance:%s yaw:%.2f Battery Volts:%.2f RobotFile:%s\n",
        Robot.alliance.toString(), Robot.yawProvider.getYaw(), RobotController.getBatteryVoltage(), getRobotType());
    logf("**********************************************************************\n");
  }

  public static double range(
      double val, double fromMin, double fromMax, double toMin, double toMax) {
    return (val - fromMin) * (toMax - toMin) / (fromMax - fromMin) + toMin;
  }

  public static double clip(double number, double min, double max) {
    if (number < min)
      return min;
    if (number > max)
      return max;
    return number;
  }

  public static double getSpeedFromTriggers(CommandXboxController controller) {
    double leftValue = controller.getLeftTriggerAxis();
    double rightValue = controller.getRightTriggerAxis();
    if (leftValue > 0.05) {
      return leftValue;
    }
    if (rightValue > 0.05) {
      return -rightValue;
    }
    return 0.0;
  }
}
