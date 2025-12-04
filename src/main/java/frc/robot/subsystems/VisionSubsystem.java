package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import javax.sound.sampled.SourceDataLine;

// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers.PoseEstimate;


public class VisionSubsystem extends SubsystemBase {
  private ShuffleboardTab tab;
  private double lastDistance;
  private DoubleSupplier rotationSupplier;

  private HttpCamera LLFeed;


  public VisionSubsystem(DoubleSupplier rotationSupplier) {
	setUpShuffleboard();
	this.rotationSupplier = rotationSupplier;
  }

  @Override
  public void periodic() {
	LimelightHelpers.SetRobotOrientation(
		getLimelightName(), rotationSupplier.getAsDouble(), 0, 0, 0, 0, 0);
  }

  /**
   * Gets the name of the limelight instance to use.
   * @return The limelight name.
   */
  public static String getLimelightName() {
	return "limelight"; //replace in a constants file somewhere
  }

  /**
   * Sets the limelight throttle to reduce processing load and manage heat output.
   * @param framesToSkip Number of frames to skip between processing. 0 = process every frame, 1 = skip every other frame, etc.
   */
  public static void setThrottle(double framesToSkip) {
	LimelightHelpers.setLimelightNTDouble(getLimelightName(), "throttle_set", framesToSkip);
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

  // public void setPipeline(String pipeline) {
  // LimelightHelpers.setPipelineIndex(getLimelightName(), pipeline.ordinal());
  // }

  public void SetIMUMode(int mode) {
	LimelightHelpers.SetIMUMode(getLimelightName(), mode);
  }

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


  public SwerveRequest alignToTag() {
	double tx = getTX();
	double ty = getTY();

	double goalX = 0.25; //random num for testing
	double goalY = 0.10;

	double xError = goalX - tx;
	double yError = goalY - ty;

	xError *= 2.0;
	yError *= 6.0;

	double yVel = MathUtil.clamp(yError, -1, 1);
	double xVel = MathUtil.clamp(xError, -1, 1);

	return new SwerveRequest.RobotCentric()
		.withVelocityX(-xVel * (DriveConstants.kMaxSpeed / 6.0))
		.withVelocityY(yVel * (DriveConstants.kMaxSpeed / 6.0))
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
		.withDeadband(DriveConstants.kMaxSpeed * 0.01)
		.withRotationalDeadband(DriveConstants.kMaxAngularRate * 0.01);
}


public SwerveRequest pointAtTag() {
	double tx = getTX();
	double ty = getTY();

	// Compute angle to tag relative to robot forward
	double angleToTag = Math.toDegrees(Math.atan2(ty, tx));

	return new SwerveRequest.RobotCentricFacingAngle()
		.withTargetDirection(Rotation2d.fromDegrees(angleToTag))
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
		.withDeadband(0)
		.withRotationalDeadband(DriveConstants.kMaxAngularRate * 0.01)
		.withVelocityX(0) // stop linear movement
		.withVelocityY(0);
}


public SwerveRequest driveAndPointAtTag() {
  double tx = getTX();
  double ty = getTY();

  //Compute angle to tag relative to robot forward
  double angleToTag = Math.toDegrees(Math.atan2(ty, tx));

  //Scale for speed
  double xVel = MathUtil.clamp(tx * 2.0, -1.0, 1.0); // X = forward/back
  double yVel = MathUtil.clamp(ty * 6.0, -1.0, 1.0); // Y = left/right

  return new SwerveRequest.RobotCentricFacingAngle()
	  .withVelocityX(-xVel * (DriveConstants.kMaxSpeed / 6.0))
	  .withVelocityY(yVel * (DriveConstants.kMaxSpeed / 6.0))
	  .withTargetDirection(Rotation2d.fromDegrees(angleToTag))
	  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
	  .withDeadband(DriveConstants.kMaxSpeed * 0.01)
	  .withRotationalDeadband(DriveConstants.kMaxAngularRate * 0.01);
}


  private void setUpShuffleboard() {
	tab = Shuffleboard.getTab("VisionApriltag");

	LLFeed = new HttpCamera(
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
