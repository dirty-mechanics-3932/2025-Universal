// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static frc.robot.utilities.Util.clip;
import static frc.robot.utilities.Util.logf;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class DrivetrainSRX extends SubsystemBase {
  public final static double MAX_VELOCITY_METERS_PER_SECOND = .1;
  public double leftStick;
  public double rightStick;
  TalonSRX talonDriveRight = new TalonSRX(Robot.config.driveRight);
  TalonSRX talonDriveRightFollow = new TalonSRX(Robot.config.driveRightFollow);
  TalonSRX talonDriveLeft = new TalonSRX(Robot.config.driveLeft);
  TalonSRX talonDriveLeftFollow = new TalonSRX(Robot.config.driveLeftFollow);
  XboxController driveController;
  private static SlewRateLimiter sLX = new SlewRateLimiter(DrivetrainSRX.MAX_VELOCITY_METERS_PER_SECOND);
  private static SlewRateLimiter sLY = new SlewRateLimiter(DrivetrainSRX.MAX_VELOCITY_METERS_PER_SECOND);
  public Double targetAngle = null;
  private double sensitivity = .8;
  private double deadZone = 0.04;


  public enum DriveTrain {
    ARCADE,
    TANK
  }

  private DriveTrain driveTrain = DriveTrain.TANK;

  public DrivetrainSRX(XboxController driveController, DriveTrain type) {
    
    logf("Start of Drive Train for SRX Subsystem\n");
    this.driveController = driveController;
    talonDriveRightFollow.follow(talonDriveRight);
    talonDriveRight.configFactoryDefault();
    talonDriveRight.setInverted(false); // pick CW versus CCW when motor controller is positive/green
    talonDriveLeft.configFactoryDefault();
    talonDriveLeftFollow.follow(talonDriveLeft);
    driveTrain = type; 
  }

  private double correctForDeadZone(double speed) {
    if (Math.abs(speed) < deadZone) {
        return 0;
    }
    return speed;
}


  private void tankDrive() {
    
    leftStick = -driveController.getLeftY();
    sLX.calculate(leftStick);
    sLY.calculate(leftStick);
    rightStick = driveController.getRightY(); // make forward stick positive
    if (Robot.count % 250 == 5) { // -1 will disable the log, set to 0 to enable log
      logf("Drive stick left:%.2f right:%.2f\n", leftStick, rightStick);
    }
    SmartDashboard.putNumber("Right Stick", rightStick);
    SmartDashboard.putNumber("Left Stick", leftStick);
    
  if (driveController.getLeftBumperButtonPressed()) {
    logf("Drive Straight start yaw:%.2f\n", Robot.yaw);
    if (targetAngle == null) { 
    targetAngle = Robot.yaw;
    }
  }
  if (driveController.getLeftBumperButtonReleased()) {
    logf("Drive Straight finish goal:%.2f yaw:%.2f\n", targetAngle, Robot.yaw);
    targetAngle = null;
  }
  if (targetAngle != null) {
    // If Drive straight active make adjustments
    driveStraight();
  }
    talonDriveLeft.set(ControlMode.PercentOutput, leftStick); 
    talonDriveRight.set(ControlMode.PercentOutput, rightStick);
  }

  private void arcadeDrive() {
        double xValue = driveController.getLeftY() * 1;
        double yValue = driveController.getRightX() * 1;
        yValue = correctForDeadZone(yValue) * sensitivity;
        xValue = correctForDeadZone(xValue) * sensitivity;

        double leftPower = yValue - xValue;
        double rightPower = yValue + xValue;

        talonDriveLeft.set(ControlMode.PercentOutput, leftPower);
        talonDriveRight.set(ControlMode.PercentOutput, rightPower);

        if (Math.abs(yValue) > .1 && Math.abs(yValue) > .1) {
            if (Robot.count % 5 == 0) {
                logf("Arcade agressive Drive Speed  r:%.3f l:%.3f\n", rightPower, leftPower);
            }
        }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Make sure that you declare this subsystem in RobotContainer.java
    if(driveTrain == DriveTrain.TANK) {
      tankDrive();
    } else  {
      arcadeDrive();
    }
  }

  void driveStraight() {
    double error = Robot.yaw - targetAngle;
    if (error > 10) {
      //  logf("!!!!! Drive Straight error too positive diff:%.1f yaw:%.1f target:%.3f\n", error,
               // Robot.yaw, targetAngle);
        error = 5;
    }
    if (error < -10) {
       // logf("!!!!! Drive Straight error too negative diff:%.1f yaw:%.1f target:%.3f\n", error,
        //        Robot.yaw, targetAngle);
        error = -5;
    }
    // Adjsut speed if too fast
    double averageJoy = (rightStick + leftStick) / 1.0;
    // If turbo mode ignore speed limit
    // if (!turboMode) {
    // if (averageJoy > .6)
    // averageJoy = .6;
    // if (averageJoy < -.6)
    // averageJoy = -.6;
    // }
    double factor = error * Math.abs(averageJoy) * 0.035; // Was 0.045
    // Log drive straight data every 2.5 seconds
    factor*=1;
    leftStick = averageJoy - factor;
    rightStick = -(averageJoy + factor);

    if (Robot.count % 12 == 0) {
        logf("Drive Straight targ:%.2f yaw:%.2f err:%.2f avg:%.2f factor:%.2f Joy:<%.2f,%.2f>\n",
                targetAngle, Robot.yaw, error,
                averageJoy, factor,  rightStick, leftStick);
    }
 
}

}