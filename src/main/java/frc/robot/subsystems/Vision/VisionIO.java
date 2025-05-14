package frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public TargetObservation latestTargetObservation =
            new TargetObservation(new Rotation2d(), new Rotation2d());
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
    }
   /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  /** Repersents the type of pose estimation emitted */
  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  public void updateInputs(VisionIOInputs inputs);

  public class VisionIONoop implements VisionIO {
    @Override
    public void updateInputs(VisionIOInputs inputs) {}
  }
}

