package frc.robot.platforms;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSRX;
import frc.robot.subsystems.LimeLightPose;

import frc.robot.subsystems.MotorSparkMax;
import frc.robot.subsystems.YawProvider;

public class BlondeMini implements RobotRunnable {
  // private final MotorSparkMax m_motorSparkMax;
  private final YawProvider m_gyroNavX;
  private final WPI_TalonSRX m_leftMotor;
  private final WPI_TalonSRX m_rightMotor;
  private final CommandXboxController m_controller;
  private final LimeLightPose m_limelight;

  private final DifferentialDriveOdometry m_drivetrainOdometry;

  public BlondeMini(CommandXboxController controller) {
    // m_motorSparkMax = new MotorSparkMax("ScoopArmMotor", 20, -1, controller, false, false);
    // m_motorSparkMax.setEncoderPosition(0);
    m_gyroNavX = new YawProvider();
    m_gyroNavX.zeroYaw();

    m_leftMotor = new WPI_TalonSRX(3);
    m_leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    m_rightMotor = new WPI_TalonSRX(2);
    m_rightMotor.setInverted(true);
    m_rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    m_limelight = new LimeLightPose();

    m_drivetrainOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);

    // new DrivetrainSRX(controller.getHID());

    this.m_controller = controller;
  }

  @Override
  public String robotName() {
    return "BlondeMini";
  }

  @Override
  public void teleopInit() {
    // m_controller.a().toggleOnTrue(manualDriveCmd());
  }

  @Override
  public void teleopPeriodic() {
    Rotation2d currentRotation = Rotation2d.fromDegrees(m_gyroNavX.getYaw());
    m_drivetrainOdometry.update(currentRotation, leftWheelDistance().in(Meters), rightWheelDistance().in(Meters));

    Pose2d currentPose = m_drivetrainOdometry.getPoseMeters();
    SmartDashboard.putNumber("OdoPoseX", currentPose.getX());
    SmartDashboard.putNumber("OdoPoseY", currentPose.getY());
  }

  // private Command manualDriveCmd() 
  //   return Commands.run(() -> {
  //     m_motorSparkMax.setPosMotionMagic(m_controller.getLeftTriggerAxis() * 10);
  //   }, m_motorSparkMax);
  // }
  private Distance wheelDiameter = Inches.of(6);
  private Dimensionless gearRatio = Value.of(1);
  private Dimensionless encoderCountPerRev = Value.of(4096);
  private Distance distancePerEncoderCount = wheelDiameter.times(Math.PI).div(encoderCountPerRev).times(gearRatio);

  private Distance leftWheelDistance() { return distancePerEncoderCount.times(m_leftMotor.getSelectedSensorPosition()); }
  private Distance rightWheelDistance() { return distancePerEncoderCount.times(m_rightMotor.getSelectedSensorPosition()); }
}
