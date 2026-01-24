// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
//import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystemNeo; //TEMP TURRET
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.QuestNavSubsystem;

public class RobotContainer {
	/* Setting up bindings for necessary control of the swerve drive platform */
	private final SwerveRequest.FieldCentric teleDrive = new SwerveRequest.FieldCentric()
			.withDeadband(DriveConstants.kDriveDeadband * DriveConstants.kMaxSpeed).withRotationalDeadband(DriveConstants.kRotationDeadband * DriveConstants.kMaxAngularRate) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

	private final SwerveRequest.FieldCentric driveFacingAngleRequest = new SwerveRequest.FieldCentric()
			.withDeadband(DriveConstants.kMaxSpeed * 0.01)
			.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final Telemetry logger = new Telemetry(DriveConstants.kMaxSpeed);

	//check if bluetooth controller is connected, if so use it
	private final CommandXboxController driverControllerBT = new CommandXboxController(IOConstants.kDriverControllerPortBT);
	private final CommandXboxController driverController = driverControllerBT.isConnected() 
		? driverControllerBT 
		: new CommandXboxController(IOConstants.kDriverControllerPortUSB);

	private final SendableChooser<Command> autoChooser;

	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

	private Field2d field2d;

	// public final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain.getPose().getRotation()::getDegrees);
	public final LimelightSubsystem limelightSubsystem = new LimelightSubsystem(() -> drivetrain.getPose().getRotation().getDegrees(), drivetrain, field2d);
	public final QuestNavSubsystem questNav = new QuestNavSubsystem(drivetrain, field2d);
	
	//public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(() -> drivetrain.getPose(), () -> drivetrain.getKinematics().toChassisSpeeds());
	public final ShooterSubsystemNeo shooterSubsystem = new ShooterSubsystemNeo(drivetrain::getPose, () -> drivetrain.getState().Speeds);
	//slew limiter object 
	SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration.in(MetersPerSecondPerSecond));
	SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration.in(MetersPerSecondPerSecond));
	SlewRateLimiter rotationLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularAcceleration.in(RadiansPerSecondPerSecond));

	public RobotContainer() {
		autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
		//constructField();

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

		// Set the logger to log to the first flashdrive plugged in
		SignalLogger.setPath("/media/sda1/");

		// Explicitly start the logger
		SignalLogger.start();
		
	}

	private void configureBindings() {
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
			// Drivetrain will execute this command periodically
			drivetrain.applyRequest(() ->
				teleDrive.withVelocityX(xLimiter.calculate(MathUtil.interpolate(1, DriveConstants.kDriveSlowModifier, driverController.getRightTriggerAxis()) * -driverController.getLeftY() * DriveConstants.kMaxSpeed)) // Drive forward with negative Y (forward)
					.withVelocityY(yLimiter.calculate(MathUtil.interpolate(1, DriveConstants.kDriveSlowModifier, driverController.getRightTriggerAxis()) * -driverController.getLeftX() * DriveConstants.kMaxSpeed)) // Drive left with negative X (left)
					.withRotationalRate(rotationLimiter.calculate(MathUtil.interpolate(1, DriveConstants.kTurnSlowModifier, driverController.getRightTriggerAxis()) * -driverController.getRightX() * DriveConstants.kMaxAngularRate)) // Drive counterclockwise with negative X (left)
			)
		);

		// Idle while the robot is disabled. This ensures the configured
		// neutral mode is applied to the drive motors while disabled.
		final var idle = new SwerveRequest.Idle();
		RobotModeTriggers.disabled().whileTrue(
			drivetrain.applyRequest(() -> idle).ignoringDisable(true)
		);

		driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
		driverController.y().whileTrue(drivetrain.applyRequest(() ->
			point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
		));

		driverController.leftBumper().whileTrue(drivetrain.applyRequest(limelightSubsystem::pointAtTag));
		driverController.rightBumper().whileTrue(drivetrain.applyRequest(limelightSubsystem::alignToTag));
		driverController.a().whileTrue(
			new DriveToPose(
				drivetrain,
				() -> new Pose2d(0, 0, new Rotation2d(0)), // always drive to origin
				driveFacingAngleRequest)
		);


		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		// reset the field-centric heading on b button press
		driverController.b().onTrue(drivetrain.runOnce(() -> drivetrain.setOperatorPerspectiveForward(drivetrain.getRotation3d().toRotation2d().rotateBy(Rotation2d.kCW_Pi_2))));// i don't know why the 90 degree rotation is necessary but it is

		driverController.start().whileTrue(limelightSubsystem.startRun(() -> LimelightSubsystem.SetIMUMode(1), limelightSubsystem::updateVisionPoseMT1)
			.finallyDo(() -> LimelightSubsystem.SetIMUMode(2))
		);

		drivetrain.registerTelemetry(logger::telemeterize);
	}

	public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

	// private void constructField() {
	// 	SmartDashboard.putData("Field", field2d);
	// }

	
}
