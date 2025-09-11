package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * FRC robot code to interface with Limelight and display pose data.
 */
public class LimeLightPose extends SubsystemBase {
    // Limelight NetworkTable
    private NetworkTable limelightTable;

    // Robot pose
    private Pose2d robotPose;
    private Field2d field;

    public LimeLightPose() {
        // Initialize connection to Limelight's NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        // Initialize Field2d for visualization
        AprilTagFieldLayout fieldLayout;
        try {
            // fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
            fieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
        } catch (Exception e) {
            fieldLayout = null;
            System.err.println("Failed to load AprilTag field layout: " + e.getMessage());
        }
        field = new Field2d();
        SmartDashboard.putData("Field", field);
        SmartDashboard.putString("FieldLayout", fieldLayout.toString());
        // Initialize pose
        robotPose = new Pose2d();
    }

    @Override
    public void periodic() {
        // Retrieve pose data from Limelight (botpose_wpiblue or botpose_wpired)
        double[] poseArray = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

        // Check if valid pose data is received
        if (poseArray.length >= 6) {
            // Extract X, Y, and rotation (yaw) from pose array
            double x = poseArray[0]; // X position in meters
            double y = poseArray[1]; // Y position in meters
            double yaw = poseArray[5]; // Yaw in degrees

            // Create Pose2d object
            robotPose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
            field.setRobotPose(robotPose);
            // Display pose data on SmartDashboard
            SmartDashboard.putNumber("Pose X (meters)", robotPose.getX());
            SmartDashboard.putNumber("Pose Y (meters)", robotPose.getY());
            SmartDashboard.putNumber("Pose Rotation (degrees)", robotPose.getRotation().getDegrees());
            SmartDashboard.putString("Robot Pose", robotPose.toString());

        } else {
            // Display error if no valid pose data
            SmartDashboard.putString("Robot Pose", "No valid pose data");
        }
    }
}
