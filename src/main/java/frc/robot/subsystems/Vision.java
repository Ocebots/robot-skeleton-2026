package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

// Source: A lot of this is ripped from YAGSL https://github.com/Yet-Another-Software-Suite/YAGSL-Example/blob/main/src%2Fmain%2Fjava%2Ffrc%2Frobot%2Fsubsystems%2Fswervedrive%2FVision.java
// Source: comments by gemini
// Source: Apriltag Filtering Tips https://www.chiefdelphi.com/t/how-can-i-filter-an-apriltag/455386
// Source: Some apriltag feilds https://www.chiefdelphi.com/t/photon-vision-apriltag-targeting/453925

public class Vision extends SubsystemBase {

  public static final PhotonCamera leftApriltagCamera = new PhotonCamera("leftApril");
  public static final PhotonCamera rightApriltagCamera = new PhotonCamera("rightApril");

  // Field dimensions in meters
  private static final double FIELD_LENGTH_METERS = 17.55;
  private static final double FIELD_WIDTH_METERS = 8.05;
  private static final double BORDER_MARGIN_METERS = 0.15;

  // Quality thresholds
  private static final double MAX_AMBIGUITY = 0.2;
  private static final double MAX_SINGLE_TAG_DISTANCE_METERS = 4.0;
  private static final double MAX_MULTI_TAG_DISTANCE_METERS = 6.5;
  private static final double MAX_Z_ERROR_METERS = 0.20;
  private static final double MAX_TILT_DEGREES = 10.0;
  private static final double MAX_TAG_ANGLE_DEGREES = 65.0;
  private static final double MAX_POSE_JUMP_METERS = 2.0;
  private static final double MAX_FRAME_AGE_SECONDS = 0.5;

  // tag IDs known to be unreliable (ignored)
  private static final Set<Integer> IGNORED_TAG_IDS = Set.of();

  public static boolean isValidPose(EstimatedRobotPose estimatedPose, Pose2d currentRobotPose) {
    Pose3d pose = estimatedPose.estimatedPose;

    // Reject frames older than 0.5 seconds
    double frameAge = Timer.getFPGATimestamp() - estimatedPose.timestampSeconds;
    if (frameAge > MAX_FRAME_AGE_SECONDS || frameAge < 0) {
      return false;
    }

    // Reject if any used tag is on the blocklist
    for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
      if (IGNORED_TAG_IDS.contains(target.getFiducialId())) {
        return false;
      }
    }

    // Field Boundary Check
    if (pose.getX() < -BORDER_MARGIN_METERS
        || pose.getX() > FIELD_LENGTH_METERS + BORDER_MARGIN_METERS
        || pose.getY() < -BORDER_MARGIN_METERS
        || pose.getY() > FIELD_WIDTH_METERS + BORDER_MARGIN_METERS) {
      return false;
    }

    // Z-Height Check: Robot must stay flat on the carpet
    if (Math.abs(pose.getZ()) > MAX_Z_ERROR_METERS) {
      return false;
    }

    // Roll and Pitch Tilt Check: Reject estimates where the robot appears tilted in 3D space
    double rollDeg = Math.abs(Units.radiansToDegrees(pose.getRotation().getX()));
    double pitchDeg = Math.abs(Units.radiansToDegrees(pose.getRotation().getY()));
    if (rollDeg > MAX_TILT_DEGREES || pitchDeg > MAX_TILT_DEGREES) {
      return false;
    }

    // Average Distance & Viewing Angle Check
    double totalDistance = 0.0;
    for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
      var transform = target.getBestCameraToTarget();
      double dist = transform.getTranslation().getNorm();
      totalDistance += dist;

      // Reject tags viewed at extreme grazing angles (causes corner distortion)
      double angleToTag = Math.abs(Math.atan2(transform.getY(), transform.getX()));
      if (Units.radiansToDegrees(angleToTag) > MAX_TAG_ANGLE_DEGREES) {
        return false;
      }
    }
    double avgDistance = totalDistance / estimatedPose.targetsUsed.size();

    // Single-Tag vs. Multi-Tag Specific Checks
    if (estimatedPose.targetsUsed.size() == 1) {
      PhotonTrackedTarget target = estimatedPose.targetsUsed.get(0);

      // Strict ambiguity check on single tags
      if (target.getPoseAmbiguity() > MAX_AMBIGUITY || target.getPoseAmbiguity() == -1) {
        return false;
      }

      // Stricter distance cap for single tags
      if (avgDistance > MAX_SINGLE_TAG_DISTANCE_METERS) {
        return false;
      }

      // Teleport / Outlier Jump Check (only applied to single tags so multi-tag can still recover)
      double jumpDistance = pose.toPose2d().getTranslation().getDistance(currentRobotPose.getTranslation());
      if (jumpDistance > MAX_POSE_JUMP_METERS) {
        return false;
      }
    } else {
      // Multi-tag distance cap
      if (avgDistance > MAX_MULTI_TAG_DISTANCE_METERS) {
        return false;
      }
    }

    return true;
  }

  // scales
  public static Matrix<N3, N1> getDynamicStdDevs(EstimatedRobotPose estimatedPose) {
    int numTags = estimatedPose.targetsUsed.size();
    double totalDist = 0.0;
    for (var target : estimatedPose.targetsUsed) {
      totalDist += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    double avgDist = totalDist / numTags;

    // Base standard deviations (meters, meters, radians)
    double xyStdDev = 0.5;
    double thetaStdDev = 0.9;

    if (numTags > 1) {
      // Trust multi-tag estimates significantly more
      xyStdDev = 0.2;
      thetaStdDev = 0.4;
    } else {
      // On single tags, trust rotation very little so pigeon dominates heading
      thetaStdDev = 999.0;
    }

    // Scale uncertainty quadratically with distance
    double distanceFactor = 1.0 + (avgDist * avgDist) / (numTags * 4.0);
    return VecBuilder.fill(xyStdDev * distanceFactor, xyStdDev * distanceFactor, thetaStdDev * distanceFactor);
  }
}

