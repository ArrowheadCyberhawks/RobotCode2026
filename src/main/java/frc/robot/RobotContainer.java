// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.rev.FlywheelSubsystemNeo;
import frc.robot.subsystems.shooter.rev.HoodSubsystemNeo;
import frc.robot.subsystems.shooter.rev.TurretSubsystemNeo;
import frc.robot.subsystems.shooter.talonfx.FlywheelSubsystem;
// import frc.robot.subsystems.shooter.talonfx.HoodSubsystem;
// import frc.robot.subsystems.shooter.talonfx.TurretSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.QuestNavSubsystem;
import frc.robot.util.HubTracker;
import frc.robot.util.HubTracker.Shift;
import frc.robot.util.field.FieldConstants;
import frc.robot.util.field.FieldZones;
import frc.robot.util.geometry.AllianceFlipUtil;
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
					DriveConstants.kRotationDeadband * DriveConstants.kMaxAngularRate.in(RadiansPerSecond)) // Add a 10% deadband
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
	() -> drivetrain.getPigeon2().getRotation2d().getDegrees(), // switched to
	// gyro-based not pose estimator
	drivetrain,
	field2d);
	public final QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem(drivetrain, field2d);

	public final HoodSubsystemNeo hood = new HoodSubsystemNeo();
	public final TurretSubsystemNeo turret = new TurretSubsystemNeo();
	public final FlywheelSubsystem flywheel = new FlywheelSubsystem();
	public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(flywheel, hood, turret);
	public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	public final HopperSubsystem hopperSubsystem = new HopperSubsystem();
	public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

	// slew limiter object
	SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration.in(MetersPerSecondPerSecond));
	SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration.in(MetersPerSecondPerSecond));
	SlewRateLimiter rotationLimiter = new SlewRateLimiter(
			DriveConstants.kMaxAngularAcceleration.in(RadiansPerSecondPerSecond));

	//create triggers to happen automatically
	//do not switch these to colons, they have mutiple methods to be recomputed
	// It will pause the robot if it runs into the trench, which is OK for now
	private final Trigger inTrench =
			new Trigger(() -> FieldZones.TRENCH().contains(drivetrain.getPose().getTranslation()
				.plus(ShooterConstants.kRobotToTurretTransform.getTranslation().toTranslation2d())));
	private final Trigger inAim =
			new Trigger(() -> FieldZones.AIM().contains(drivetrain.getPose().getTranslation()
				.plus(ShooterConstants.kRobotToTurretTransform.getTranslation().toTranslation2d())));
	private final Trigger inLeftPass =
			new Trigger(() -> FieldZones.LEFTPASS().contains(drivetrain.getPose().getTranslation()
				.plus(ShooterConstants.kRobotToTurretTransform.getTranslation().toTranslation2d())));
	private final Trigger inRightPass =
			new Trigger(() -> FieldZones.RIGHTPASS().contains(drivetrain.getPose().getTranslation()
				.plus(ShooterConstants.kRobotToTurretTransform.getTranslation().toTranslation2d())));

	Command shootMode = new ShootCommand(shooterSubsystem, hopperSubsystem, inTrench::getAsBoolean);

	public RobotContainer() {

		// Register Named Commands for PathPlanner BEFORE creating any autos
		registerNamedCommands();
		registerEventMarkers();

		constructField();
		configureBindings();
		configureTriggers();
		WebServer.start(5800, Filesystem.getDeployDirectory().getPath()); // elastic
		// and the hood/turret won't adjust based on distance!

		// TODO: simplify the number of suppliers given by just giving robotrelative speeds and pose
		ShotCalculator.getInstance().setPoseSupplier(drivetrain::getPose);
		ShotCalculator.getInstance().setRobotRelativeVelocitySupplier(() -> drivetrain.getState().Speeds);
		ShotCalculator.getInstance().setFieldVelocitySupplier(
			() -> ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getPose().getRotation())
		);

		// Reconfigure AutoBuilder to also reset Quest pose when auto starts
		// Maybe don't do this if the LL pose is closer to the pose given by autobuilder
		drivetrain.configureAutoBuilderWithPoseReset((pose) -> {
			drivetrain.resetPose(pose);
			questNavSubsystem.resetPose(pose);
		});

		autoChooser = new LoggedDashboardChooser<>("Auto/Selected", AutoBuilder.buildAutoChooser("LeftSwipe"));

		// Warmup PathPlanner to avoid Java pauses
		CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

		// Set the logger to log to the first flashdrive plugged in
		SignalLogger.setPath("/media/sda1/");

		// Explicitly start the logger
		SignalLogger.start();
		SmartDashboard.putNumber("ResetX", 0);
		SmartDashboard.putNumber("ResetY", 0);
		SmartDashboard.putNumber("ResetTheta", 0);
		SmartDashboard.putData("Reset Pose", new InstantCommand(() -> drivetrain.resetPose(
				new Pose2d(
						new Translation2d(
								SmartDashboard.getNumber("ResetX", 0.0),
								SmartDashboard.getNumber("ResetY", 0.0)),
						new Rotation2d(
								SmartDashboard.getNumber("ResetTheta", 0.0))))));

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
							* MathUtil.applyDeadband(-driverController.getLeftY(), DriveConstants.kDriveDeadband)
							* DriveConstants.kMaxSpeed.in(MetersPerSecond)))
				.withVelocityY(
					yLimiter.calculate(
						MathUtil.interpolate(1,
							DriveConstants.kDriveSlowModifier,
								driverController.getRightTriggerAxis())
									* MathUtil.applyDeadband(-driverController.getLeftX(), DriveConstants.kDriveDeadband)
									* DriveConstants.kMaxSpeed.in(MetersPerSecond))) // Drive left with negative X (left)
				.withRotationalRate(
					rotationLimiter.calculate(
						MathUtil.interpolate(1,
							DriveConstants.kTurnSlowModifier,
								driverController.getRightTriggerAxis())
									* MathUtil.applyDeadband(-driverController.getRightX(), DriveConstants.kRotationDeadband)
									* DriveConstants.kMaxAngularRate.in(RadiansPerSecond))) // Drive
				));
		
		// Left bumper - Toggle intake
		driverController.leftBumper().toggleOnTrue(intakeSubsystem.runEnd(
			() -> intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.RUN),
			() -> intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.IDLE)
		));

		driverController.rightBumper().toggleOnTrue(shootMode);

		// Idle while the robot is disabled. This ensures the configured
		// neutral mode is applied to the drive motors while disabled.
		final var idle = new SwerveRequest.Idle();
		RobotModeTriggers.disabled().whileTrue(
				drivetrain.applyRequest(() -> idle).ignoringDisable(true));

		// driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
		// driverController.y().whileTrue(drivetrain.applyRequest(() -> point
		// .withModuleDirection(new Rotation2d(-driverController.getLeftY(),
		// -driverController.getLeftX()))));

		driverController.povUp().or(manipulatorController.povUp())
			.whileTrue(climberSubsystem.runClimberDown());
		driverController.povDown().or(manipulatorController.povDown())
			.whileTrue(climberSubsystem.runClimberUp());
		driverController.povLeft().or(manipulatorController.povLeft())
			.onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.STOW)));
		driverController.povRight().or(manipulatorController.povRight()).onTrue(shooterSubsystem.trenchCommand()); // assuming this is a trench related manual control
		
		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		// reset the field-centric heading on b button press
		driverController.b().onTrue(drivetrain.runOnce(() -> drivetrain.setOperatorPerspectiveForward(
				drivetrain.getState().Pose.getRotation()
		)));

		// driverController.start()
		// .whileTrue(limelightSubsystem
		// .startRun(() -> LimelightSubsystem.SetIMUMode(1),
		// () -> limelightSubsystem.updateVisionPoseMT1(true))
		// .finallyDo(() -> LimelightSubsystem.SetIMUMode(3)));

		// driverController.back()
		// 		.whileTrue(questNavSubsystem.run(() -> {
		// 			ChassisSpeeds speeds = drivetrain.getState().Speeds;
		// 			boolean isSlow = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.25
		// 					&& Math.abs(speeds.omegaRadiansPerSecond) < 0.1;
		// 			if (limelightSubsystem.hasTarget() && isSlow) {
		// 				questNavSubsystem.resetPose(limelightSubsystem.getMegaTag2Pose2dFromLimelight());
		// 			}
		// 		}));

		driverController.start().or(manipulatorController.start()).whileTrue(Commands.run(()-> 
			questNavSubsystem.resetPose(AllianceFlipUtil.apply(new Pose2d(3.500, 4.040, new Rotation2d(Math.PI/2)))))
			.andThen(() -> drivetrain.resetPose(AllianceFlipUtil.apply(new Pose2d(3.500, 4.040, new Rotation2d(Math.PI/2))))));

		drivetrain.registerTelemetry(logger::telemeterize);

		driverController.leftTrigger()
			.or(manipulatorController.leftTrigger())
				.whileTrue(Commands.runEnd(
						() -> intakeSubsystem.setIntakeState(IntakeState.RUN),
						() -> intakeSubsystem.setIntakeState(IntakeState.IDLE)));

		driverController.x().and(driverController.leftTrigger())
				.or(manipulatorController.x().and(manipulatorController.leftTrigger()))
				.whileTrue(
						intakeSubsystem.runEnd(
								() -> intakeSubsystem.setIntakeState(IntakeState.REVERSE),
								() -> intakeSubsystem.setIntakeState(IntakeState.IDLE)));

		// hopper controls
		manipulatorController.leftBumper().whileTrue(
				hopperSubsystem.runEnd(
						() -> hopperSubsystem.setHopperState(HopperState.ON),
						() -> hopperSubsystem.setHopperState(HopperState.IDLE)));

		manipulatorController.x().and(manipulatorController.leftBumper())
				.whileTrue(
						hopperSubsystem.runEnd(
								() -> hopperSubsystem.setHopperState(HopperState.REVERSE),
								() -> hopperSubsystem.setHopperState(HopperState.IDLE)));
		
		manipulatorController.rightStick().whileTrue(
			shooterSubsystem.manualTurretCommand(() -> Rotation2d.fromDegrees(turret.getSetpoint().getDegrees() + -Math.pow(manipulatorController.getRightX(), 5) * 2.0))
			.alongWith(shooterSubsystem.manualHoodCommand(() -> hood.getSetpoint().plus(Rotation2d.fromDegrees(Math.pow(-manipulatorController.getRightY(), 3) * 0.5))))
		);
		manipulatorController.leftStick().toggleOnTrue(
			shooterSubsystem.manualFlywheelCommand(() -> flywheel.getSetpoint()
								.plus(RotationsPerSecond.of(
										MathUtil.applyDeadband(-manipulatorController.getLeftY(), 0.05)))));
	}

	private void configureTriggers() {
		// should probably put sc.setTarget before this so I don't have get the instance each time

		inAim.onTrue(Commands.runOnce(() -> {
			ShotCalculator sc = ShotCalculator.getInstance();
			sc.setTarget(FieldConstants.Hub.topCenterPoint.toTranslation2d());
		}));

		inLeftPass.onTrue(Commands.runOnce(() -> {
			ShotCalculator sc = ShotCalculator.getInstance();
			sc.setTarget(FieldConstants.Corners.left.toTranslation2d());
		}));

		inRightPass.onTrue(Commands.runOnce(() -> {
			ShotCalculator sc = ShotCalculator.getInstance();
			sc.setTarget(FieldConstants.Corners.right.toTranslation2d());
		}));
		
		// inTrench.whileTrue(shooterSubsystem.trenchCommand());
		// inTrench.whileTrue(Commands.runOnce(() -> shooterSubsystem.requestState(ShooterSubsystem.ShooterState.TRENCH)));
	}

	/**
	 * Register named commands for use in PathPlanner autos.
	 * These commands can be referenced by name in the PathPlanner GUI.
	 */
	private void registerNamedCommands() {
		// Intake Commands
		NamedCommands.registerCommand("IntakeOn",
				Commands.runOnce(
						() -> intakeSubsystem.setIntakeState(IntakeState.RUN)));
		
		// CommandScheduler.getInstance().schedule(Commands.run(() -> intakeSubsystem.setIntakeState(IntakeState.RUN)));

		NamedCommands.registerCommand("IntakeOff",
				Commands.runOnce(() -> intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.IDLE)));

		NamedCommands.registerCommand("IntakeStow",
				Commands.runOnce(() -> intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.STOW)));
		
		NamedCommands.registerCommand("ShooterCommand", shootMode);

		NamedCommands.registerCommand("ShooterAim", shooterSubsystem.aimCommand());

		NamedCommands.registerCommand("HopperOn",
				Commands.runOnce(() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.ON)));

		NamedCommands.registerCommand("HopperOff",
				Commands.runOnce(() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.IDLE)));

		// Shooter Commands - Stop all shooting motors
		NamedCommands.registerCommand("StopShoot",
			Commands.sequence(
			Commands.runOnce(() -> shooterSubsystem.stop()),
			Commands.runOnce(() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.IDLE)))
		);
	}

	private void registerEventMarkers() {
		new EventTrigger("IntakeOn").onTrue(
			Commands.runOnce(() -> intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.RUN)));
		new EventTrigger("IntakeStow").onTrue(
			Commands.runOnce(() -> intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.STOW)));
		new EventTrigger("IntakeOff").onTrue(
			Commands.runOnce(() -> intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.IDLE)));
		new EventTrigger("IntakeTrench").onTrue(
			Commands.runOnce(() -> intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.TRENCH)));

		new EventTrigger("ShooterAim").onTrue(shooterSubsystem.aimCommand());
		new EventTrigger("HopperOn").onTrue(
				Commands.runOnce(() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.ON)));
		new EventTrigger("HopperOff").onTrue(
				Commands.runOnce(() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.IDLE)));

		// Shooter Commands - Stop all shooting motors
		new EventTrigger("ShooterOff").onTrue(
			Commands.sequence(
			Commands.runOnce(() -> shooterSubsystem.stop()),
			Commands.runOnce(() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.IDLE))));

		
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
