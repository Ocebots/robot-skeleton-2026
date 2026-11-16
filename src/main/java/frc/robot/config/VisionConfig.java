package frc.robot.config;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConfig {

    public static final PhotonPoseEstimator.PoseStrategy STRATEGY = PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;
    public static final Transform3d LEFT_CAMERA_POSITION = new Transform3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation3d(0.0, 0.0, 0.0));
    public static final Transform3d RIGHT_CAMERA_POSITION = new Transform3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation3d(0.0, 0.0, 0.0));
    public static PhotonPoseEstimator photonPoseEstimatorLeft = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), STRATEGY, LEFT_CAMERA_POSITION);
    public static PhotonPoseEstimator photonPoseEstimatorRight = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), STRATEGY, RIGHT_CAMERA_POSITION);
}
