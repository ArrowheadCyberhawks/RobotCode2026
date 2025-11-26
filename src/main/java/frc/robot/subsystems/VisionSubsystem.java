package frc.robot.subsystems;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;
import java.util.function.DoubleSupplier;


public class VisionSubsystem extends SubsystemBase {
  private ShuffleboardTab tab;
  private double lastDistance;
  private DoubleSupplier rotationSupplier;

  private HttpCamera LLFeed;


  public VisionSubsystem(DoubleSupplier rotationSupplier) {
    setUpShuffleboard();
    this.rotationSupplier = rotationSupplier;
  }

  private String getLimelightName() {
    return "limelight";
  }

  public double getTX() {
    return LimelightHelpers.getTX(getLimelightName());
  }

  public double getTY() {
    return LimelightHelpers.getTY(getLimelightName());
  }

  public Pose2d getPose2dFromLimelight() {
    return LimelightHelpers.getBotPose2d(getLimelightName());
  }

  public Pose2d getMegaTag2Pose2dFromLimelight() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(getLimelightName()).pose;
  }

  public Pose3d getPose3dTargetSpaceFromLimelight() {
    return (LimelightHelpers.getBotPose3d_TargetSpace(getLimelightName()));
  }

  public PoseEstimate getPoseEstimate() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(getLimelightName());
  }

  public double getLatencyCapture() {
    return LimelightHelpers.getLatency_Capture(getLimelightName());
  }

  public double getLatencyPipeline() {
    return LimelightHelpers.getLatency_Pipeline(getLimelightName());
  }

  public boolean hasTarget() {
    return LimelightHelpers.getTV(getLimelightName());
  }

  public double getAprilTagId() {
    return LimelightHelpers.getFiducialID(getLimelightName());
  }

//   public void setPipeline(String pipeline) {
//     LimelightHelpers.setPipelineIndex(getLimelightName(), pipeline.ordinal());
//   }


  public double getDistanceToTarget() {
    if (!hasTarget()) {
      return lastDistance;
    }
    double cameraHeight = 22;
    double targetHeight = 56.375;
    double heightDiff = targetHeight - cameraHeight;
    double cameraAngle = 23;
    double theta = Math.toRadians(cameraAngle + getTY());
    lastDistance = heightDiff / Math.tan(theta);
    return lastDistance;
  }

  @Override
  public void periodic() {
    // Add 180 degrees because camera is mounted on opposite side
    LimelightHelpers.SetRobotOrientation(
        getLimelightName(), rotationSupplier.getAsDouble() + 180, 0, 0, 0, 0, 0);
  }

  private void setUpShuffleboard() {
    tab = Shuffleboard.getTab("VisionApriltag");

    LLFeed =
        new HttpCamera(
            getLimelightName(),
            "http://10.7.6.11:5800/stream.mjpg"); // TODO check if this IP is correct
    tab.add("Limelight Feed", LLFeed).withWidget("Camera Stream").withPosition(0, 0).withSize(4, 4);

    tab.addDouble("Tx", () -> this.getTX());
    tab.addDouble("Ty", () -> this.getTY());
    tab.addDouble("Distance", () -> this.getDistanceToTarget());
    tab.addDouble("Latency Capture", () -> this.getLatencyCapture());
    tab.addDouble("Latency Pipeline", () -> this.getLatencyPipeline());
    tab.addBoolean("Has Target", () -> this.hasTarget());

    tab.addDouble("April Tag", () -> this.getAprilTagId());
  }
}
