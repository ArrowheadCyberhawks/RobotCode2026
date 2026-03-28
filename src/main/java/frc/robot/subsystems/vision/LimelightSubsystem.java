package frc.robot.subsystems.vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
// import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldObjects;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;


public class LimelightSubsystem extends SubsystemBase {
  private double lastDistance;
  private DoubleSupplier rotationSupplier;
  private Supplier<Rotation3d> rotation3dSupplier;
  private BooleanSupplier useLimelight;
  private CommandSwerveDrivetrain drivetrain;
  private final Field2d field2d;
  private final double averageTagAreaMT2 = 0.7;


  public LimelightSubsystem(DoubleSupplier rotationSupplier, BooleanSupplier useLimelight, CommandSwerveDrivetrain drivetrain, Field2d field2d) {
	this.rotationSupplier = rotationSupplier;
	this.rotation3dSupplier = () -> new Rotation3d();
	this.useLimelight = useLimelight;
	this.drivetrain = drivetrain;
	this.field2d = field2d;
  }

    public LimelightSubsystem(Supplier<Rotation3d> rotation3dSupplier, BooleanSupplier useLimelight, CommandSwerveDrivetrain drivetrain, Field2d field2d) {
	this.rotation3dSupplier = rotation3dSupplier;
	this.rotationSupplier = () -> rotation3dSupplier.get().getZ();
	this.useLimelight = useLimelight;
	this.drivetrain = drivetrain;
	this.field2d = field2d;
  }

  @Override
  public void periodic() {
	updateRobotOrientation();
	// updateFullRobotOrientation();
	if (useLimelight.getAsBoolean()) {
		updateVisionPoseMT2();
	}
	// updateVisionPoseMT1(true);
	updateField(true);
	// if(DriverStation.isDisabled()) {
	// 	updateVisionPoseMT1(true);
	// }
    Logger.recordOutput("Limelight/Enabled?", useLimelight.getAsBoolean());
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

  /**
   * Updates the robot orientation in the limelight based on the supplied rotation supplier.
   */
  public void updateRobotOrientation() {
	LimelightHelpers.SetRobotOrientation(
		getLimelightName(), rotationSupplier.getAsDouble(), 0, 0, 0, 0, 0);
  }

  public void updateFullRobotOrientation() {
	LimelightHelpers.SetRobotOrientation(
		getLimelightName(), rotation3dSupplier.get().getZ(), 0.0, rotation3dSupplier.get().getY(), 0.0, rotation3dSupplier.get().getX(), 0.0);
  }

  public void updateVisionPoseMT1(boolean rotationOnly) {
		LimelightHelpers.PoseEstimate limelightMeasurementMT1 = getPoseEstimateMT1();
		
		if (limelightMeasurementMT1 != null && !limelightMeasurementMT1.pose.equals(Pose2d.kZero)) {
			Vector<N3> measurementStdDevs;
			if (rotationOnly) {
				measurementStdDevs = VecBuilder.fill(999999,999999,0.5);
			} else {
				measurementStdDevs = VecBuilder.fill(.5,.5,1);
			}
			drivetrain.addVisionMeasurement(limelightMeasurementMT1.pose, Utils.fpgaToCurrentTime(limelightMeasurementMT1.timestampSeconds), measurementStdDevs);
		}
	}

	public void updateVisionPoseMT2() {
		LimelightHelpers.PoseEstimate limelightMeasurementMT2 = getPoseEstimateMT2();
		
		if (limelightMeasurementMT2 == null
                || limelightMeasurementMT2.pose.equals(Pose2d.kZero)
                || limelightMeasurementMT2.tagCount == 0
                || limelightMeasurementMT2.rawFiducials == null
                || limelightMeasurementMT2.rawFiducials.length == 0) {
            return;
        }
			// not doing an OR here bc I want to be able to test them individually
			boolean rejectUpdate = true;

			if (limelightMeasurementMT2.tagCount >= 2) {
				rejectUpdate = false;
			}

			if (limelightMeasurementMT2.tagCount == 1 && limelightMeasurementMT2.avgTagArea > averageTagAreaMT2) {
				rejectUpdate = false;
			}

			if (Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) > Units.degreesToRadians(450)) { // replace 1.0 with appropriate threshold
				rejectUpdate = true;
			}

			if (!rejectUpdate) {
				drivetrain.addVisionMeasurement(
					limelightMeasurementMT2.pose,
					Utils.fpgaToCurrentTime(limelightMeasurementMT2.timestampSeconds),
					VecBuilder.fill(.6,.6,9999999));
			}

			// horrible inefficient garbage telemetry code
			// SmartDashboard.putNumber("Vision Heading", drivetrain.getPose().getRotation().getDegrees());
			// SmartDashboard.putNumberArray("Robot Pose", new double[] { limelightMeasurementMT2.pose.getX(),
			// limelightMeasurementMT2.pose.getY(), limelightMeasurementMT2.pose.getRotation().getDegrees() });
	}

  public double getTX() {
	return LimelightHelpers.getTX(getLimelightName());
  }

  public double getTY() {
	return LimelightHelpers.getTY(getLimelightName());
  }

  public Pose3d getPose3dTargetSpaceFromLimelight() {
	return (LimelightHelpers.getBotPose3d_TargetSpace(getLimelightName()));
  }

  public PoseEstimate getPoseEstimateMT1() {
	return LimelightHelpers.getBotPoseEstimate_wpiBlue(getLimelightName());
  }

  public PoseEstimate getPoseEstimateMT2() {
	return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(getLimelightName());
  }

  public double getLatency() {
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

  public void setPipeline(int pipeline) {
  LimelightHelpers.setPipelineIndex(getLimelightName(), pipeline);
  }

  /**
   * Sets the IMU mode of the limelight.
   * @param mode The IMU mode to set. 0 = external yaw only, 1 = sync internal yaw to external yaw, 2 = use internal yaw only.
   */
  public static void SetIMUMode(int mode) {
	LimelightHelpers.SetIMUMode(getLimelightName(), mode);
	LimelightHelpers.SetIMUAssistAlpha(getLimelightName(), 0.001);
  }

  public double getDistanceToTarget() {
	if (!hasTarget()) {
	  return lastDistance;
	}
	double cameraHeight = 22; // ✨ magic numbers ✨
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

	xError = MathUtil.clamp(xError, -1, 1);
	yError = MathUtil.clamp(yError, -1, 1);

	return new SwerveRequest.RobotCentric()
		.withVelocityX(DriveConstants.kMaxSpeed.times(6.0).unaryMinus().times(xError))
		.withVelocityY(DriveConstants.kMaxSpeed.times(6.0).times(yError))
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
		.withDeadband(DriveConstants.kMaxSpeed.times(0.01))
		.withRotationalDeadband(DriveConstants.kMaxAngularRate.times(0.01));
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
		.withRotationalDeadband(DriveConstants.kMaxAngularRate.times(0.01))
		.withVelocityX(0) // stop linear movement
		.withVelocityY(0);
}

// public double getDistanceToObject() { //in m
// 	Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_INTAKE)
// 			.plus(Rotation2d.fromDegrees(getTYDeg(INTAKE_LL_NAME)));
// 	if (angleToGoal.getDegrees() <= 0) {
// 		double distance = (HEIGHT_FROM_GROUND_METERS_INTAKE - NOTE_HEIGHT)
// 				/ Math.tan(Math.abs(angleToGoal.getRadians()));
// 		// SmartDashboard.putNumber("limelight distance", distance);
// 		return distance;
// 	} else {
// 		// SmartDashboard.putNumber("limelight distance", -1);
// 		return -1;
// 	}
// }

public SwerveRequest driveAndPointAtTag() {
  double tx = getTX();
double ty = getTY();

double goalX = 0.25; //random num for testing
double goalY = 0.10;

double xError = goalX - tx;
double yError = goalY - ty;

xError *= 2.0;
yError *= 6.0;

xError = MathUtil.clamp(xError, -1, 1);
yError = MathUtil.clamp(yError, -1, 1);

  //Compute angle to tag relative to robot forward
  double angleToTag = Math.toDegrees(Math.atan2(ty, tx));

  //Scale for speed
  double xVel = MathUtil.clamp(tx * 2.0, -1.0, 1.0); // X = forward/back
  double yVel = MathUtil.clamp(ty * 6.0, -1.0, 1.0); // Y = left/right

  return new SwerveRequest.RobotCentricFacingAngle()
	  .withVelocityX(DriveConstants.kMaxSpeed.div(6.0).times(-xVel))
	  .withVelocityY(DriveConstants.kMaxSpeed.div(6.0).times(yVel))
	  .withTargetDirection(Rotation2d.fromDegrees(angleToTag))
	  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
	  .withDeadband(DriveConstants.kMaxSpeed.times(0.01))
	  .withRotationalDeadband(DriveConstants.kMaxAngularRate.times(0.01));
}

  private void updateField(boolean useMegaTag2) {
		LimelightHelpers.PoseEstimate limelightMeasurement = useMegaTag2 ? getPoseEstimateMT2() : getPoseEstimateMT1();

		if (limelightMeasurement != null && !limelightMeasurement.pose.equals(Pose2d.kZero)) {
			field2d.getObject(FieldObjects.LIMELIGHT).setPose(limelightMeasurement.pose);
		}
  }
}
