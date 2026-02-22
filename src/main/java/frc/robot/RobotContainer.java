// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.rev.FlywheelSubsystemNeo;
import frc.robot.subsystems.shooter.rev.HoodSubsystemNeo;
import frc.robot.subsystems.shooter.rev.TurretSubsystemNeo;
import frc.robot.subsystems.shooter.talonfx.FlywheelSubsystem;
import frc.robot.subsystems.shooter.talonfx.HoodSubsystem;
import frc.robot.subsystems.shooter.talonfx.TurretSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.QuestNavSubsystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.HubTracker;
import frc.robot.util.HubTracker.Shift;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem.HopperState;

public class RobotContainer {
	/* Setting up bindings for necessary control of the swerve drive platform */
	private final SwerveRequest.FieldCentric teleDrive = new SwerveRequest.FieldCentric()
			// .withDeadband(DriveConstants.kDriveDeadband *
			// DriveConstants.kMaxSpeed.in(MetersPerSecond))
			.withRotationalDeadband(
					DriveConstants.kRotationDeadband * DriveConstants.kMaxAngularRate.in(RadiansPerSecond)) // Add a 10%
																											// deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

	private final SwerveRequest.FieldCentric driveFacingAngleRequest = new SwerveRequest.FieldCentric()
			// .withDeadband(DriveConstants.kMaxSpeed.in(MetersPerSecond) * 0.01)
			.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final Telemetry logger = new Telemetry(DriveConstants.kMaxSpeed.in(MetersPerSecond));

	// check if bluetooth controller is connected, if so use it
	private final CommandXboxController driverControllerBT = new CommandXboxController(
			IOConstants.kDriverControllerPortBT);
	private final CommandXboxController driverController = driverControllerBT.isConnected()
			? driverControllerBT
			: new CommandXboxController(IOConstants.kDriverControllerPortUSB);

	private final CommandXboxController manipulatorControllerBT = new CommandXboxController(
			IOConstants.kManipulatorControllerPortBT);
	private final CommandXboxController manipulatorController = manipulatorControllerBT.isConnected()
			? manipulatorControllerBT
			: new CommandXboxController(IOConstants.kManipulatorControllerPortUSB);

	private final LoggedDashboardChooser<Command> autoChooser;

	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

	private Field2d field2d = new Field2d();

	// public final VisionSubsystem visionSubsystem = new
	// VisionSubsystem(drivetrain.getPose().getRotation()::getDegrees);
	public final LimelightSubsystem limelightSubsystem = new LimelightSubsystem(
			() -> drivetrain.getPigeon2().getRotation2d().getDegrees(), // switched to gyro-based not pose estimator
			drivetrain,
			field2d);
	public final QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem(drivetrain, field2d);

	public final HoodSubsystemNeo hood = new HoodSubsystemNeo();
	public final TurretSubsystemNeo turret = new TurretSubsystemNeo();
	public final FlywheelSubsystem flywheel = new FlywheelSubsystem();
	// public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(
	// 		flywheel,
	// 		hood,
	// 		turret
	// 	);

	// Intake subsystem
	public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	
	// Hopper subsystem
	public final HopperSubsystem hopperSubsystem = new HopperSubsystem();
	
	// slew limiter object
	SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration.in(MetersPerSecondPerSecond));
	SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration.in(MetersPerSecondPerSecond));
	SlewRateLimiter rotationLimiter = new SlewRateLimiter(
			DriveConstants.kMaxAngularAcceleration.in(RadiansPerSecondPerSecond));
	


	public RobotContainer() {
		constructField();
		configureBindings();
		turret.manualResetTurretEncoder(-Math.PI/2);
		WebServer.start(5800, Filesystem.getDeployDirectory().getPath()); // elastic

		// Configure ShotCalculator with robot pose supplier
		// This is critical - without this, the calculator thinks the robot is always at
		// (0,0)
		// and the hood/turret won't adjust based on distance!
		ShotCalculator.getInstance().setPoseSupplier(drivetrain::getPose);
		// Field-relative velocities (used for turret velocity calculation)
		ShotCalculator.getInstance().setRobotRelativeVelocitySupplier(() -> drivetrain.getState().Speeds);

		// Robot-relative velocities (used for pose prediction during shot flight)
		ShotCalculator.getInstance().setFieldVelocitySupplier(() -> {
			ChassisSpeeds robotSpeeds = drivetrain.getState().Speeds;
			Rotation2d robotRotation = drivetrain.getPose().getRotation();

			// Convert robot-relative to field-relative by rotating velocity vector
			double cosAngle = robotRotation.getCos();
			double sinAngle = robotRotation.getSin();

			return new ChassisSpeeds(
					robotSpeeds.vxMetersPerSecond * cosAngle - robotSpeeds.vyMetersPerSecond * sinAngle,
					robotSpeeds.vxMetersPerSecond * sinAngle + robotSpeeds.vyMetersPerSecond * cosAngle,
					robotSpeeds.omegaRadiansPerSecond);
		});

		// TODO: Add velocity suppliers for moving shot compensation if needed

		// Reconfigure AutoBuilder to also reset Quest pose when auto starts
		drivetrain.configureAutoBuilderWithPoseReset((pose) -> {
			drivetrain.resetPose(pose);
			questNavSubsystem.resetPose(pose);
		});

		// Register Named Commands for PathPlanner BEFORE creating any autos
		registerNamedCommands();

		autoChooser = new LoggedDashboardChooser<>("Auto/Selected", AutoBuilder.buildAutoChooser("Left1Cycle"));

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
				drivetrain.applyRequest(() -> teleDrive
						.withVelocityX(
								xLimiter.calculate(
										MathUtil.interpolate(1,
												DriveConstants.kDriveSlowModifier,
												driverController.getRightTriggerAxis())
												* MathUtil.applyDeadband(-driverController.getLeftY(),
														DriveConstants.kDriveDeadband)
												* DriveConstants.kMaxSpeed.in(MetersPerSecond))) // Drive forward with
																									// negative Y
																									// (forward)
						.withVelocityY(
								yLimiter.calculate(
										MathUtil.interpolate(1,
												DriveConstants.kDriveSlowModifier,
												driverController.getRightTriggerAxis())
												* MathUtil.applyDeadband(-driverController.getLeftX(),
														DriveConstants.kDriveDeadband)
												* DriveConstants.kMaxSpeed.in(MetersPerSecond))) // Drive left with
																									// negative X (left)
						.withRotationalRate(
								rotationLimiter.calculate(
										MathUtil.interpolate(1,
												DriveConstants.kTurnSlowModifier,
												driverController.getRightTriggerAxis())
												* MathUtil.applyDeadband(-driverController.getRightX(),
														DriveConstants.kRotationDeadband)
												* DriveConstants.kMaxAngularRate.in(RadiansPerSecond))) // Drive
																										// counterclockwise
																										// with negative
																										// X (left)
				));


		driverController.leftTrigger().onTrue(Commands.runEnd(
			() -> intakeSubsystem.setIntakeState(IntakeState.RUN),
			() -> intakeSubsystem.setIntakeState(IntakeState.IDLE)
		));
		
		// Left bumper - Toggle intake between RUN and OFF
		// driverController.leftTrigger().onTrue(Commands.runOnce(() -> {
		// 	IntakeConstants.IntakeState currentState = intakeSubsystem.getIntakeState();
		// 	if (currentState == IntakeConstants.IntakeState.RUN) {
		// 		intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.IDLE);
		// 	} else {
		// 		intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.RUN);
		// 	}
		// }));

		// Right bumper - Toggle shooter: Start aiming sequence if idle, or stop if already aiming/shooting
		// driverController.rightBumper().onTrue(Commands.either(
		// 	// If shooter is in AIM or SHOOT state, stop everything
		// 	Commands.sequence(
		// 		Commands.runOnce(shooterSubsystem::stop),
		// 		Commands.runOnce(() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.IDLE))
		// 	),
		// 	// If shooter is IDLE or STRAIGHT, start aiming sequence
		// 	Commands.sequence(
		// 		// First, start aiming
		// 		Commands.runOnce(shooterSubsystem::startAiming),
		// 		// Wait until shooter is ready to shoot
		// 		Commands.waitUntil(shooterSubsystem::isReadyToShoot),
		// 		// Once ready, turn on the hopper to feed the ball
		// 		Commands.runOnce(() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.ON))
		// 	),
		// 	// Condition: true if shooter is already aiming or shooting
		// 	() -> shooterSubsystem.getState() == ShooterSubsystem.ShooterState.AIM || 
		// 	      shooterSubsystem.getState() == ShooterSubsystem.ShooterState.SHOOT
		// ));
		


		//TEMP CODE
		driverController.rightBumper().whileTrue(hood.trackTarget().alongWith(flywheel.trackTarget()));
		driverController.y().whileTrue(flywheel.diagnosePhase());
		//driverController.y().whileTrue(turret.trackTarget());

		// Smart target selection based on field zone: D-pad down will set the target
		// to the hub if we're in our alliance zone; otherwise choose left/right
		// corner based on which side of the field we're on.
		// TODO: Find a place to put this in so that it runs all the time (some periodic?)
		driverController.povLeft().onTrue(Commands.runOnce(() -> {
			ShotCalculator sc = ShotCalculator.getInstance();
			Pose2d pose = drivetrain.getPose();
			double x = pose.getX();
			double y = pose.getY();

			// Check if we're in the neutral zone (past neutralZoneNear)
			if (x < FieldConstants.LinesVertical.neutralZoneNear) {
				// We're in our alliance zone - shoot at the hub
				sc.setTarget(FieldConstants.Hub.topCenterPoint.toTranslation2d());
			} else {
				// We're in neutral/opponent zone - choose corner based on Y position
				if (y > FieldConstants.LinesHorizontal.center) {
					// Above center - shoot at left corner
					sc.setTarget(FieldConstants.Corners.left.toTranslation2d());
				} else {
					// Below center - shoot at right corner
					sc.setTarget(FieldConstants.Corners.right.toTranslation2d());
				}
			}
		}));

		// Idle while the robot is disabled. This ensures the configured
		// neutral mode is applied to the drive motors while disabled.
		final var idle = new SwerveRequest.Idle();
		RobotModeTriggers.disabled().whileTrue(
				drivetrain.applyRequest(() -> idle).ignoringDisable(true));

		// driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
		// driverController.y().whileTrue(drivetrain.applyRequest(() -> point
		// 		.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

		// driverController.leftBumper().whileTrue(drivetrain.applyRequest(limelightSubsystem::pointAtTag));
		driverController.povUp().whileTrue(
				new DriveToPose(
						drivetrain,
						() -> new Pose2d(0, 0, new Rotation2d(0)), // always drive to origin
						driveFacingAngleRequest));

		driverController.povDown().onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.STOW)));

		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		// reset the field-centric heading on b button press
		driverController.b().onTrue(drivetrain.runOnce(() -> drivetrain.setOperatorPerspectiveForward(
				drivetrain.getState().Pose.getRotation() // drivetrain.getPose().getRotation()
		)));

		// driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

		driverController.start()
				.whileTrue(limelightSubsystem
						.startRun(() -> LimelightSubsystem.SetIMUMode(1),
								() -> limelightSubsystem.updateVisionPoseMT1(true))
						.finallyDo(() -> LimelightSubsystem.SetIMUMode(3)));

		driverController.back().whileTrue(questNavSubsystem.run(() -> questNavSubsystem.resetPose(drivetrain.getPose())));

		// Manipulator controller - Shooter state machine controls
		// A button: Set shooter to IDLE (stop all subsystems)
		//manipulatorController.a().onTrue(Commands.runOnce(shooterSubsystem::stop));


		// B button: Set shooter to STRAIGHT (turret straight, tracking flywheel/hood)
		//manipulatorController.b().onTrue(Commands.runOnce(shooterSubsystem::aimStraight));

		// X button: Start AIM sequence (all subsystems track target, auto-transitions
		// to SHOOT when ready)
		//manipulatorController.x().onTrue(Commands.runOnce(shooterSubsystem::startAiming));

		drivetrain.registerTelemetry(logger::telemeterize);

		// driverController.leftBumper().whileTrue(
		// 	hopperSubsystem.runEnd(
		// 		() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.ON),
		// 		() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.IDLE)
		// 	)
		// );

		driverController.leftBumper().onTrue(Commands.runOnce(() -> {
			HopperState currentState = hopperSubsystem.getHopperState();
			if (currentState == HopperState.ON) {
				hopperSubsystem.setHopperState(HopperState.IDLE);
			} else {
				hopperSubsystem.setHopperState(HopperState.ON);
			}
		}));
	}

	/**
	 * Register named commands for use in PathPlanner autos.
	 * These commands can be referenced by name in the PathPlanner GUI.
	 */
	private void registerNamedCommands() {
		// Intake Commands
		NamedCommands.registerCommand("IntakeOn", 
			Commands.runOnce(() -> intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.RUN))
		);
		
		NamedCommands.registerCommand("IntakeOff", 
			Commands.runOnce(() -> intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.IDLE))
		);

		// TODO: Uncomment when shooter subsystem is enabled
		// Shooter Commands - Start shooting sequence
		// NamedCommands.registerCommand("StartShoot", 
		// 	Commands.sequence(
		// 		// First, start aiming
		// 		Commands.runOnce(() -> shooterSubsystem.startAiming()),
		// 		// Wait until shooter is ready to shoot
		// 		Commands.waitUntil(() -> shooterSubsystem.isReadyToShoot()),
		// 		// Once ready, turn on the hopper to feed the ball
		// 		Commands.runOnce(() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.ON))
		// 	)
		// );

		// Shooter Commands - Stop all shooting motors
		// NamedCommands.registerCommand("StopShoot", 
		// 	Commands.sequence(
		// 		Commands.runOnce(() -> shooterSubsystem.stop()),
		// 		Commands.runOnce(() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.IDLE))
		// 	)
		// );
	}

	public Command getAutonomousCommand() {
		/* Run the path selected from the auto chooser */
		return autoChooser.get();
	}

	private void constructField() {
		SmartDashboard.putData("Field", field2d);
	}

	public void updateField2d() {
		field2d.setRobotPose(drivetrain.getPose());

		// Add ShotCalculator target to field2d
		Translation2d target = ShotCalculator.getInstance().getTarget();
		Pose2d targetPose = new Pose2d(target, new Rotation2d());
		field2d.getObject("ShotTarget").setPose(targetPose);
	}

}
