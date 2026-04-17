// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser; //Lebron

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem.HopperState;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystemNeo;
// import frc.robot.subsystems.shooter.talonfx.HoodSubsystem;
// import frc.robot.subsystems.shooter.talonfx.TurretSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystemNeo;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.QuestNavSubsystem;
import frc.robot.util.HubTracker;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.field.FieldConstants;
import frc.robot.util.field.FieldZones;
import frc.robot.util.geometry.AllianceFlipUtil;

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
	
	private Field2d field2d = new Field2d();
	private final Telemetry logger = new Telemetry(DriveConstants.kMaxSpeed.in(MetersPerSecond), field2d);

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
	public final QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem(drivetrain, field2d);
	public final LimelightSubsystem limelightSubsystem = new LimelightSubsystem(
		() -> drivetrain.getPigeon2().getRotation2d().getDegrees() + (AllianceFlipUtil.shouldFlip() ? 180 : 0),
		// () -> drivetrain.getPose().getRotation().getDegrees(),
		() -> !questNavSubsystem.useQuest(),
		drivetrain,
		field2d
	);

	public final HoodSubsystemNeo hood = new HoodSubsystemNeo();
	public final TurretSubsystem turret = new TurretSubsystem();
	public final FlywheelSubsystem flywheel = new FlywheelSubsystem();
	public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(flywheel, hood, turret);
	public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	public final HopperSubsystem hopperSubsystem = new HopperSubsystem();
	public final LEDSubsystem ledSubsystem = new LEDSubsystem(
		intakeSubsystem::getIntakeState,
		shooterSubsystem::getState);

	// public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

	// slew limiter object
	SlewRateLimiter xLimiter = new SlewRateLimiter(
		DriveConstants.kMaxAcceleration.in(MetersPerSecondPerSecond),
		-DriveConstants.kMaxDeceleration.in(MetersPerSecondPerSecond),
		0.0);
	SlewRateLimiter yLimiter = new SlewRateLimiter(
		DriveConstants.kMaxAcceleration.in(MetersPerSecondPerSecond),
		-DriveConstants.kMaxDeceleration.in(MetersPerSecondPerSecond),
		0.0);
	SlewRateLimiter rotationLimiter = new SlewRateLimiter(
			DriveConstants.kMaxAngularAcceleration.in(RadiansPerSecondPerSecond),
			-DriveConstants.kMaxAngularDeceleration.in(RadiansPerSecondPerSecond),
			0.0);

	//create triggers to happen automatically

	private final Trigger inTrench =
			new Trigger(() -> FieldZones.TRENCH().contains(drivetrain.getPose().getTranslation()
				.plus(ShooterConstants.kRobotToTurretTransform.getTranslation().toTranslation2d())));
	private final Trigger inTower =
			new Trigger(() -> FieldZones.TOWER().contains(drivetrain.getPose().getTranslation()
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
	private final Trigger inPass = inLeftPass.or(inRightPass);

	Command shootMode = new ShootCommand(shooterSubsystem, hopperSubsystem, inTrench::getAsBoolean, inTower::getAsBoolean, inPass::getAsBoolean);

	Trigger endOfShift =
        new Trigger(() ->
            HubTracker.timeRemainingInCurrentShift()
                .map(timeRemaining -> {
                    double seconds = timeRemaining.in(Seconds);
                    return seconds >= 0.0 && seconds <= 8.0;
                })
                .orElse(false)
        );

	private Command shootLeft;
	private Command shootRight;
	private Command farHubClearLeft;
	private Command farHubClearRight;
	private Command hubClearLeft;
	private Command hubClearRight;


	public RobotContainer() {
		
		// Register Named Commands for PathPlanner BEFORE creating any autos
		registerNamedCommands();
		registerEventMarkers();

		// Reconfigure AutoBuilder to also reset Quest pose when auto starts
		// Maybe don't do this if the LL pose is closer to the pose given by autobuilder

		configureTriggers();
		drivetrain.configureAutoBuilderWithPoseReset((pose) -> {
			drivetrain.resetPose(pose);
			questNavSubsystem.resetPose(pose);
		});

		registerPathfinding();
		constructField();
		configureBindings();

		WebServer.start(5800, Filesystem.getDeployDirectory().getPath()); // elastic
		Pathfinding.setPathfinder(new LocalADStarAK());

		// TODO: simplify the number of suppliers given by just giving robotrelative speeds and pose
		ShotCalculator.getInstance().setPoseSupplier(drivetrain::getPose);
		ShotCalculator.getInstance().setRobotRelativeVelocitySupplier(() -> drivetrain.getState().Speeds);
		ShotCalculator.getInstance().setFieldVelocitySupplier(
			() -> ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getPose().getRotation())
		);

		//Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback(
			(activePath) -> {
			Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
		});
    	PathPlannerLogging.setLogTargetPoseCallback(
			(targetPose) -> {
			Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
		});



		//TODO only add certain autos to the chooser and flip them to have various starting positions
		autoChooser = new LoggedDashboardChooser<>("Auto/Selected");

		buildAutonomousCommands();

		autoChooser.onChange((Command selected) -> {
			// reset pose to the starting pose of the selected auto
			if (selected instanceof PathPlannerAuto) {
				PathPlannerAuto auto = (PathPlannerAuto) selected;
				Pose2d initialPose = auto.getStartingPose();
				initialPose = AllianceFlipUtil.apply(initialPose);
				drivetrain.resetPose(initialPose);
				questNavSubsystem.resetPose(initialPose);
			}
		});

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

	private void buildAutonomousCommands() {
		Command leftDoubleSwipe = new PathPlannerAuto("left-trench-2swipe");
		Command leftDoubleSwipeBump = new PathPlannerAuto("left-bump-2swipe");
		Command leftDoubleSwipeDepotBump = new PathPlannerAuto("left-bump-2swipe-depot");
		Command rightDoubleSwipe = new PathPlannerAuto("left-trench-2swipe", true);
		Command rightDoubleSwipeBump = new PathPlannerAuto("left-bump-2swipe", true);
		Command rightRiskPass = new PathPlannerAuto("right-pass-corner");
		Command leftbumpdisruptor = new PathPlannerAuto("disruptor-left-bump-2swipe-depot");
		Command middleDepot = new PathPlannerAuto("depot-auto");


		autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
		autoChooser.addOption("LeftDoubleSwipe", leftDoubleSwipe);
		autoChooser.addOption("leftDoubleSwipeBump", leftDoubleSwipeBump);
		autoChooser.addOption("leftDoubleSwipeDepotBump", leftDoubleSwipeDepotBump);
		autoChooser.addOption("RightDoubleSwipe", rightDoubleSwipe);
		autoChooser.addOption("RightDoubleSwipeBump", rightDoubleSwipeBump);
		autoChooser.addOption("RightRiskPass", rightRiskPass);
		autoChooser.addOption("LeftBumpDisruptor", leftbumpdisruptor);
		autoChooser.addOption("MiddleDepot", middleDepot);
	}

	private void configureBindings() {
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
			// Drivetrain will execute this command periodically
			drivetrain.applyRequest(() -> teleDrive
				// picks the slower trigger if both are pressed
				.withVelocityX(
					xLimiter.calculate(
						MathUtil.interpolate(DriveConstants.kDriveSlowModifier,
								DriveConstants.kDriveFastModifier,
								(driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis() + 1) / 2) // combine triggers into one axis
									* MathUtil.applyDeadband(-driverController.getLeftY(), DriveConstants.kDriveDeadband)
									* DriveConstants.kMaxSpeed.in(MetersPerSecond)
						))//left trigger slow, right trigger fast
				.withVelocityY(
					yLimiter.calculate(
						MathUtil.interpolate(DriveConstants.kDriveSlowModifier,
								DriveConstants.kDriveFastModifier,
								(driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis() + 1) / 2)
									* MathUtil.applyDeadband(-driverController.getLeftX(), DriveConstants.kDriveDeadband)
									* DriveConstants.kMaxSpeed.in(MetersPerSecond))) // Drive left with negative X (left)
				.withRotationalRate(
					rotationLimiter.calculate(
						MathUtil.interpolate(DriveConstants.kTurnSlowModifier,
								DriveConstants.kTurnFastModifier,
								(driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis() + 1) / 2)
									* MathUtil.applyDeadband(-driverController.getRightX(), DriveConstants.kRotationDeadband)
									* DriveConstants.kMaxAngularRate.in(RadiansPerSecond))) // Drive
				));

		//shooterSubsystem.setDefaultCommand(shooterSubsystem.trenchCommand());
		
		// Left bumper - Toggle intake (use start/end so we don't re-assert state every scheduler cycle)
		driverController.leftBumper().toggleOnTrue(Commands.startEnd(
			() -> intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.RUN),
			() -> intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.IDLE),
			intakeSubsystem
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

		// driverController.povUp().or(manipulatorController.povUp())
		// 	.whileTrue(climberSubsystem.runClimberDown());
		// driverController.povDown().or(manipulatorController.povDown())
		// 	.whileTrue(climberSubsystem.runClimberUp());
		
		
		manipulatorController.povLeft()
			.onTrue(intakeSubsystem.runOnce(() -> intakeSubsystem.setIntakeState(IntakeConstants.IntakeState.STOW)));

		manipulatorController.povRight().onTrue(shooterSubsystem.trenchCommand());
		
		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		// driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		// driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		// driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		// driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		// reset the field-centric heading on b button press
		driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.setOperatorPerspectiveForward(
				drivetrain.getState().Pose.getRotation()
		)));

		//Protection/Defense Mode
		driverController.x().onTrue(
			intakeSubsystem.runOnce(() -> intakeSubsystem.setIntakeState(IntakeState.STOW))
			//.andThen(ledSubsystem)
		);

		driverController.a().onTrue(
			intakeSubsystem.runOnce(() -> {
				IntakeState current = intakeSubsystem.getIntakeState();
				if (current == IntakeState.REVERSE) {
					intakeSubsystem.setIntakeState(IntakeState.RUN);
				} else {
					intakeSubsystem.setIntakeState(IntakeState.REVERSE);
				}
			})
		);

		driverController.b().onTrue(shooterSubsystem.trenchCommand());

		driverController.povLeft().whileTrue(shootLeft);
		driverController.povRight().whileTrue(shootRight);

		// Bind hub clear paths based on driver station location
		int dsLocation = DriverStation.getLocation().orElse(0);
		if (dsLocation == 1 || dsLocation == 2 || dsLocation == 0) {
			// Stations 1 & 2 (and unknown 0) use LEFT variants
			driverController.povUp().whileTrue(farHubClearLeft);
			driverController.povDown().whileTrue(hubClearLeft);
		} else if (dsLocation == 3) {
			// Station 3 uses RIGHT variants
			driverController.povUp().whileTrue(farHubClearRight);
			driverController.povDown().whileTrue(hubClearRight);
		}


		driverController.start()
		.whileTrue(Commands.run(()-> 
			drivetrain.resetPose(AllianceFlipUtil.apply(new Pose2d(3.500, 4.040, new Rotation2d(Math.PI)))))
			// .andThen(drivetrain.runOnce(() -> drivetrain.setOperatorPerspectiveForward(
			// 	drivetrain.getState().Pose.getRotation().plus(new Rotation2d(Math.PI)))))
			.alongWith(new InstantCommand(() -> questNavSubsystem.resetPose(AllianceFlipUtil.apply(new Pose2d(3.500, 4.040, new Rotation2d(Math.PI)))))));

		driverController.back().whileTrue(Commands.run(() -> {
			limelightSubsystem.updateVisionPoseMT1(true);
			Pose2d limelightPose = limelightSubsystem.getPoseEstimateMT1().pose;
			if (limelightPose != null) {
				drivetrain.resetPose(limelightPose);
				questNavSubsystem.resetPose(limelightPose);
			}
		}));

		drivetrain.registerTelemetry(logger::telemeterize);

	manipulatorController.leftTrigger()
		.whileTrue(Commands.startEnd(
			() -> intakeSubsystem.setIntakeState(IntakeState.RUN),
			() -> intakeSubsystem.setIntakeState(IntakeState.IDLE),
			intakeSubsystem));

	manipulatorController.x().and(manipulatorController.leftTrigger())
		.whileTrue(
			Commands.startEnd(
				() -> intakeSubsystem.setIntakeState(IntakeState.REVERSE),
				() -> intakeSubsystem.setIntakeState(IntakeState.IDLE),
				intakeSubsystem));

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

		manipulatorController.b().whileTrue(
			intakeSubsystem.manualPivotCommand(() -> MathUtil.applyDeadband(manipulatorController.getLeftX(), 0.05))
		);
		manipulatorController.start().whileTrue(
			Commands.run(intakeSubsystem::resetPivotEncoder, intakeSubsystem)
		);
		manipulatorController.back().whileTrue(
			Commands.run(hood::zeroHood)
		);

		
		endOfShift.whileTrue(
			Commands.run(
					() -> {
						double intensity = getShiftRumbleIntensity();
						driverController.setRumble(RumbleType.kBothRumble, intensity);
						manipulatorController.setRumble(RumbleType.kBothRumble, intensity);
					})
				.finallyDo(
					() -> {
						driverController.setRumble(RumbleType.kBothRumble, 0.0);
						manipulatorController.setRumble(RumbleType.kBothRumble, 0.0);
					})
		);
	}

	private void configureTriggers() {
		// TODO: should probably put sc.setTarget before this so I don't have get the instance each time
		inAim.whileTrue(Commands.runOnce(() -> {
			ShotCalculator sc = ShotCalculator.getInstance();
			sc.setTarget(FieldConstants.Hub.topCenterPoint.toTranslation2d());
		}));

		inLeftPass.whileTrue(Commands.runOnce(() -> {
			ShotCalculator sc = ShotCalculator.getInstance();
			sc.setTarget(FieldConstants.Corners.left.toTranslation2d());
		}));

		inRightPass.whileTrue(Commands.runOnce(() -> {
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

		NamedCommands.registerCommand("ShooterTrench", shooterSubsystem.trenchCommand());

		NamedCommands.registerCommand("ShooterOff", 
			Commands.sequence(
				shooterSubsystem.idle(),
				Commands.runOnce(() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.IDLE))
			)
		);

		NamedCommands.registerCommand("ShooterOn", shooterSubsystem.aimCommand());

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

		new EventTrigger("ShooterAim").onTrue(shootMode);
		new EventTrigger("HopperOn").onTrue(
				Commands.runOnce(() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.ON)));
		new EventTrigger("HopperOff").onTrue(
				Commands.runOnce(() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.IDLE)));

		new EventTrigger("ShooterOn").onTrue(
			shooterSubsystem.aimCommand());

		// Shooter Commands - Stop all shooting motors
		new EventTrigger("ShooterOff").onTrue(
			Commands.sequence(
				shooterSubsystem.idle(),
				Commands.runOnce(() -> hopperSubsystem.setHopperState(HopperSubsystem.HopperState.IDLE))));

		new EventTrigger("ShooterTrench").onTrue(shooterSubsystem.trenchCommand());
		
	}

	private void registerPathfinding() {
		// Left Shoot
		Command leftShootCommand;
		try {
			PathPlannerPath path = PathPlannerPath.fromPathFile("LeftShoot");
			PathConstraints constraints = new PathConstraints(
					3.0, 4.0,
					Units.degreesToRadians(540), Units.degreesToRadians(720));
			leftShootCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
		} catch (Exception e) {
			DriverStation.reportError("Failed to load PathPlanner path 'LeftShoot': " + e.getMessage(), e.getStackTrace());
			leftShootCommand = Commands.none();
		}
		this.shootLeft = leftShootCommand;

		// Right Shoot
		Command rightShootCommand;
		try {
			PathPlannerPath path = PathPlannerPath.fromPathFile("RightShoot");
			PathConstraints constraints = new PathConstraints(
					3.0, 4.0,
					Units.degreesToRadians(540), Units.degreesToRadians(720));
			rightShootCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
		} catch (Exception e) {
			DriverStation.reportError("Failed to load PathPlanner path 'RightShoot': " + e.getMessage(), e.getStackTrace());
			rightShootCommand = Commands.none();
		}
		this.shootRight = rightShootCommand;

		// FarHubClearLeft
		Command farHubClearLeftCommand;
		try {
			PathPlannerPath path = PathPlannerPath.fromPathFile("FarHubClearLeft");
			PathConstraints constraints = new PathConstraints(
					3.0, 4.0,
					Units.degreesToRadians(540), Units.degreesToRadians(720));
			farHubClearLeftCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
		} catch (Exception e) {
			DriverStation.reportError("Failed to load PathPlanner path 'FarHubClearLeft': " + e.getMessage(), e.getStackTrace());
			farHubClearLeftCommand = Commands.none();
		}
		this.farHubClearLeft = farHubClearLeftCommand;

		// FarHubClearRight
		Command farHubClearRightCommand;
		try {
			PathPlannerPath path = PathPlannerPath.fromPathFile("FarHubClearRight");
			PathConstraints constraints = new PathConstraints(
					3.0, 4.0,
					Units.degreesToRadians(540), Units.degreesToRadians(720));
			farHubClearRightCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
		} catch (Exception e) {
			DriverStation.reportError("Failed to load PathPlanner path 'FarHubClearRight': " + e.getMessage(), e.getStackTrace());
			farHubClearRightCommand = Commands.none();
		}
		this.farHubClearRight = farHubClearRightCommand;

		// HubClearLeft
		Command hubClearLeftCommand;
		try {
			PathPlannerPath path = PathPlannerPath.fromPathFile("HubClearLeft");
			PathConstraints constraints = new PathConstraints(
					3.0, 4.0,
					Units.degreesToRadians(540), Units.degreesToRadians(720));
			hubClearLeftCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
		} catch (Exception e) {
			DriverStation.reportError("Failed to load PathPlanner path 'HubClearLeft': " + e.getMessage(), e.getStackTrace());
			hubClearLeftCommand = Commands.none();
		}
		this.hubClearLeft = hubClearLeftCommand;

		// HubClearRight
		Command hubClearRightCommand;
		try {
			PathPlannerPath path = PathPlannerPath.fromPathFile("HubClearRight");
			PathConstraints constraints = new PathConstraints(
					3.0, 4.0,
					Units.degreesToRadians(540), Units.degreesToRadians(720));
			hubClearRightCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);
		} catch (Exception e) {
			DriverStation.reportError("Failed to load PathPlanner path 'HubClearRight': " + e.getMessage(), e.getStackTrace());
			hubClearRightCommand = Commands.none();
		}
		this.hubClearRight = hubClearRightCommand;

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

	/**
	 * Compute rumble intensity based on time remaining in the current shift.
	 * Intensity is 0 outside the last 8 seconds of a shift and follows 1 / t
	 * (clamped to 1.0) as time t (in seconds) approaches 0.
	 */
	private double getShiftRumbleIntensity() {
		var remainingOpt = HubTracker.timeRemainingInCurrentShift();
		if (remainingOpt.isEmpty()) {
			return 0.0;
		}

		double secondsRemaining = MathUtil.clamp(remainingOpt.get().in(Seconds), 1.0, 8.0);

		double intensity = 1.0 / secondsRemaining;
		return MathUtil.clamp(intensity, 0.0, 1.0);
	}

}
