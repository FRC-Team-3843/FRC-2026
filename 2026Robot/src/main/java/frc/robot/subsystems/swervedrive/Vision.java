package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Vision subsystem stub for PhotonVision integration.
 * Infrastructure is ready - add cameras to the Cameras enum when hardware is mounted.
 * Uses 2025 field layout until 2026 layout is available.
 */
public class Vision {

  /**
   * AprilTag field layout. Update to k2026 when available.
   */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
      AprilTagFields.k2025ReefscapeAndyMark);

  private final double maximumAmbiguity = 0.25;

  public VisionSystemSim visionSim;
  private Supplier<Pose2d> currentPose;
  private Field2d field2d;

  /**
   * Construct Vision with pose supplier and field display.
   *
   * @param currentPose Pose supplier from SwerveDrive.
   * @param field       Field2d from SwerveDrive for visualization.
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field) {
    this.currentPose = currentPose;
    this.field2d = field;

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (Cameras c : Cameras.values()) {
        c.addToVisionSim(visionSim);
      }
    }
  }

  /**
   * Get the pose of an AprilTag with a robot offset applied.
   *
   * <p>Useful for calculating target poses relative to AprilTags. For example, to score
   * a game piece at a position 0.5m in front of tag 7, call:
   * <pre>getAprilTagPose(7, new Transform2d(new Translation2d(0.5, 0), new Rotation2d()))</pre>
   *
   * @param aprilTag The AprilTag ID to query.
   * @param robotOffset Transform to apply to the tag pose (robot frame).
   * @return The resulting field pose after applying the offset.
   * @throws RuntimeException if the tag ID is not found in the field layout.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }
  }

  /**
   * Update the swerve drive's pose estimation with vision measurements from all cameras.
   *
   * <p>This method should be called periodically (typically in a subsystem's periodic() or
   * a dedicated vision subsystem). It queries all configured cameras for AprilTag detections,
   * calculates robot pose estimates, and feeds them to the swerve drive's pose estimator
   * with appropriate standard deviations based on tag count and distance.
   *
   * <p>In simulation, this also updates the vision sim with the robot's current pose.
   *
   * @param swerveDrive The swerve drive subsystem to update with vision measurements.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
      visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
    }
    for (Cameras camera : Cameras.values()) {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isPresent()) {
        var pose = poseEst.get();
        swerveDrive.addVisionMeasurement(
            pose.estimatedPose.toPose2d(),
            pose.timestampSeconds,
            camera.curStdDevs);
      }
    }
  }

  /**
   * Get the estimated global pose from a specific camera.
   *
   * <p>Queries the camera for AprilTag detections and uses PhotonVision's pose estimator
   * to calculate the robot's field pose. Returns empty if no tags are visible or the
   * estimate is unreliable (high ambiguity, single tag too far, etc.).
   *
   * <p>In simulation mode, also updates the debug field visualization with the estimate.
   *
   * @param camera The camera enum entry to query.
   * @return Optional containing the estimated pose, or empty if unavailable/unreliable.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    if (Robot.isSimulation()) {
      Field2d debugField = visionSim.getDebugField();
      poseEst.ifPresentOrElse(
          est -> debugField.getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
          () -> debugField.getObject("VisionEstimation").setPoses());
    }
    return poseEst;
  }

  /**
   * Get distance from the robot to a specific AprilTag.
   *
   * <p>Calculates 2D distance (ignoring height difference) from the robot's current
   * odometry pose to the specified tag. Useful for driver assistance features like
   * "distance to scoring target" displays.
   *
   * @param id AprilTag ID.
   * @return Distance in meters, or -1.0 if tag ID is not found in field layout.
   */
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  public VisionSystemSim getVisionSim() {
    return visionSim;
  }

  /**
   * Camera enum - add entries here when cameras are physically mounted on the robot.
   *
   * Example entry (uncomment and modify when adding a camera):
   * <pre>
   * FRONT_CAM("front",
   *           new Rotation3d(0, Math.toRadians(-15), 0),
   *           new Translation3d(Units.inchesToMeters(12), 0, Units.inchesToMeters(8)),
   *           VecBuilder.fill(4, 4, 8),
   *           VecBuilder.fill(0.5, 0.5, 1))
   * </pre>
   */
  enum Cameras {
    // No cameras defined yet - add entries above when hardware is ready
    ;

    public final Alert latencyAlert;
    public final PhotonCamera camera;
    public final PhotonPoseEstimator poseEstimator;
    private final Matrix<N3, N1> singleTagStdDevs;
    private final Matrix<N3, N1> multiTagStdDevs;
    private final Transform3d robotToCamTransform;
    public Matrix<N3, N1> curStdDevs;
    public Optional<EstimatedRobotPose> estimatedRobotPose;
    public PhotonCameraSim cameraSim;
    public List<PhotonPipelineResult> resultsList = new ArrayList<>();
    private double lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    /**
     * Camera constructor with physical mounting parameters and measurement confidence values.
     *
     * @param name Camera name (must match PhotonVision UI).
     * @param robotToCamRotation Camera rotation relative to robot center (pitch, yaw, roll).
     * @param robotToCamTranslation Camera position relative to robot center (X forward, Y left, Z up).
     * @param singleTagStdDevs Standard deviations [x, y, theta] for single-tag measurements.
     * @param multiTagStdDevsMatrix Standard deviations [x, y, theta] for multi-tag measurements.
     */
    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
            Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix) {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);
      camera = new PhotonCamera(name);
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);
      poseEstimator = new PhotonPoseEstimator(
          Vision.fieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;

      if (Robot.isSimulation()) {
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        cameraProp.setCalibError(0.25, 0.08);
        cameraProp.setFPS(30);
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
      }
    }

    /**
     * Add this camera to the vision simulation environment.
     *
     * @param systemSim VisionSystemSim instance to add the camera to.
     */
    public void addToVisionSim(VisionSystemSim systemSim) {
      if (Robot.isSimulation()) {
        systemSim.addCamera(cameraSim, robotToCamTransform);
      }
    }

    /**
     * Get the estimated robot pose from this camera.
     *
     * <p>Internally updates with all unread results from the camera since the last call.
     *
     * @return Optional containing the most recent estimated pose, or empty if unavailable.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      updateUnreadResults();
      return estimatedRobotPose;
    }

    /**
     * Fetch and process all unread results from the camera.
     *
     * <p>Uses a debounce mechanism to avoid processing results too frequently and
     * sorts results by timestamp to ensure chronological processing.
     */
    private void updateUnreadResults() {
      double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
      double currentTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
      double debounceTime = Milliseconds.of(15).in(Seconds);
      for (PhotonPipelineResult result : resultsList) {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }
      if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime))
          && (currentTimestamp - lastReadTimestamp) >= debounceTime) {
        resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
        lastReadTimestamp = currentTimestamp;
        resultsList.sort((a, b) -> a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1);
        if (!resultsList.isEmpty()) {
          updateEstimatedGlobalPose();
        }
      }
    }

    /**
     * Process all queued camera results and update the estimated robot pose.
     *
     * <p>Iterates through all results, updating standard deviations based on tag count
     * and distance for each measurement.
     */
    private void updateEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList) {
        visionEst = poseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
    }

    /**
     * Calculate appropriate standard deviations for a vision measurement based on tag count and distance.
     *
     * <p>Standard deviation represents measurement uncertainty:
     * <ul>
     *   <li>Multi-tag estimates are more trustworthy (lower std dev)</li>
     *   <li>Measurements get less trustworthy as distance increases (higher std dev)</li>
     *   <li>Single-tag estimates beyond 4 meters are rejected (set to MAX_VALUE)</li>
     * </ul>
     *
     * @param estimatedPose The estimated pose from PhotonVision.
     * @param targets List of detected AprilTags in this measurement.
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        curStdDevs = singleTagStdDevs;
      } else {
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        for (var tgt : targets) {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) {
            continue;
          }
          numTags++;
          avgDist += tagPose.get().toPose2d().getTranslation()
              .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
          curStdDevs = singleTagStdDevs;
        } else {
          avgDist /= numTags;
          if (numTags > 1) {
            estStdDevs = multiTagStdDevs;
          }
          if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }
  }
}
