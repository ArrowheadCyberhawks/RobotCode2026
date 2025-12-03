// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
	private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
																						// max angular velocity

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

	private final SwerveRequest.FieldCentricFacingAngle driveFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(MaxSpeed * 0.01)
			.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final Telemetry logger = new Telemetry(MaxSpeed);

	private final CommandXboxController joystick = new CommandXboxController(0);

	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

	// public final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain.getPose().getRotation()::getDegrees);
	private VisionSubsystem visionSubsystem = new VisionSubsystem(() -> drivetrain.getPose().getRotation().getDegrees());
	//private QuestNavSubsystem questNav = new QuestNavSubsystem(drivetrain);


	public RobotContainer() {
		configureBindings();
		visionSubsystem.SetIMUMode(1);

		// Set the logger to log to the first flashdrive plugged in
		SignalLogger.setPath("/media/sda1/");

		// Explicitly start the logger
		SignalLogger.start();
	}

	public void periodic() {
		updateVisionPose();
	}

	private void configureBindings() {
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
			// Drivetrain will execute this command periodically
			drivetrain.applyRequest(() ->
				drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
					.withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
					.withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
			)
		);

		// Idle while the robot is disabled. This ensures the configured
		// neutral mode is applied to the drive motors while disabled.
		final var idle = new SwerveRequest.Idle();
		RobotModeTriggers.disabled().whileTrue(
			drivetrain.applyRequest(() -> idle).ignoringDisable(true)
		);

		joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
		joystick.y().whileTrue(drivetrain.applyRequest(() ->
			point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
		));

		joystick.leftBumper().whileTrue(drivetrain.applyRequest(visionSubsystem::pointAtTag));
		joystick.rightBumper().whileTrue(drivetrain.applyRequest(visionSubsystem::alignToTag));
		joystick.a().whileTrue(
			new DriveToPose(
				drivetrain,
				() -> new Pose2d(10, 5, new Rotation2d(0)), // always drive to origin
				driveFacingAngleRequest)
		);


		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		// reset the field-centric heading on b button press
		joystick.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

		drivetrain.registerTelemetry(logger::telemeterize);
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}

	public void updateVisionPose() {
		LimelightHelpers.PoseEstimate limelightMeasurement = visionSubsystem.getPoseEstimate();
		System.out.println("current robot pose: " + drivetrain.getPose());
		if (!limelightMeasurement.pose.equals(new Pose2d())) {
			drivetrain.addVisionMeasurement(limelightMeasurement.pose, Utils.fpgaToCurrentTime(limelightMeasurement.timestampSeconds));//, VecBuilder.fill(.5,.5,9999999));
			// horrible inefficient garbage telemetry code
			SmartDashboard.putNumber("Vision Heading", drivetrain.getPose().getRotation().getDegrees());
			SmartDashboard.putNumberArray("Robot Pose", new double[] { limelightMeasurement.pose.getX(),
			limelightMeasurement.pose.getY(), limelightMeasurement.pose.getRotation().getDegrees() });
		}
	}

}
