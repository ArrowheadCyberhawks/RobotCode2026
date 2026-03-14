package frc.robot.subsystems.shooter.rev;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.LoggedTunableNumber;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
/**
 * Turret implementation using REV CANSparkMax (NEO).
 * Uses REV onboard PID controller for position control.
 */
public class TurretSubsystemNeo extends SubsystemBase {
	private final SparkMax turretMotor;
	private final RelativeEncoder encoder;
	private final SparkClosedLoopController turretController;
	private ShotCalculator shotCalculator = ShotCalculator.getInstance();

	private SparkMaxConfig turretConfig;	
	private Rotation2d targetRotation = new Rotation2d();

	public TurretSubsystemNeo() {
		this(ShooterConstants.kTurretMotorId);
	}

	public TurretSubsystemNeo(int motorId) {
		turretMotor = new SparkMax(motorId, MotorType.kBrushless);
		encoder = turretMotor.getEncoder();
		turretController = turretMotor.getClosedLoopController();

		turretConfig = new SparkMaxConfig();

		configureTurret();
	}

	/**
	 * Set target azimuth for the turret. The controller will rotate the turret to
	 * the specified angle.
	 * 
	 * @param targetTurretAngle Target angle as a Rotation2d. Zero is forward, positive is CCW, negative is CW.
	 */
	public void setSetpoint(Rotation2d targetTurretAngle) {
		// Clamp targetAngle to the configured soft limit range
		targetRotation = Rotation2d.fromRadians(
			MathUtil.clamp(
				targetTurretAngle.getRadians(),
				ShooterConstants.turretMinAngle.get(),
				ShooterConstants.turretMaxAngle.get())
		);
	}

	public Rotation2d getSetpoint() {
		return targetRotation;
	}

	public Rotation2d getTurretRotation() {
		try {
			return Rotation2d.fromRadians(encoder.getPosition());
		} catch (Exception e) {
			return new Rotation2d();
		}
	}

	public void setTurretVoltage(double volts) {
		turretMotor.setVoltage(volts);
	}

	public void stopTurret() {
		targetRotation = getTurretRotation(); // Update target to current position to prevent jumps when resuming control
		turretController.setSetpoint(targetRotation.getRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
	}

	/**
	 * Check if the turret is at its goal position.
	 * 
	 * @param toleranceRadians The tolerance in radians
	 * @return true if the turret is within tolerance of the target
	 */
	public boolean atGoal(double toleranceRadians) {
		double currentRadians = getTurretRotation().getRadians();

		// Handle angle wrapping for shortest distance
		double error = Math.abs(Rotation2d.fromRadians(currentRadians)
				.minus(targetRotation)
				.getRadians());

		return error <= toleranceRadians;
	}

	/**
	 * Check if the turret is at its goal position using default tolerance.
	 * 
	 * @return true if the turret is within 3 degrees (~0.052 radians) of the target
	 */
	public boolean isAtGoal() {
		return atGoal(Math.toRadians(3.0)); // 3 degree default tolerance
	}

	public void resetTurretEncoder() {
		// Get current position in radians
		double currentRadians = encoder.getPosition();
		// Wrap to [-π, π] range to match ShotCalculator
		double wrappedRadians = Math.atan2(Math.sin(currentRadians), Math.cos(currentRadians));
		encoder.setPosition(wrappedRadians);
	}

	public void manualResetTurretEncoder(double rotations) {
		encoder.setPosition(rotations);
	}

	@Override
	public void periodic() {
		// Update PID constants if they've changed in NetworkTables
		updateTurretPID();
		
		turretController.setSetpoint(targetRotation.getRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0);

		// Log telemetry
		Logger.recordOutput("Turret/Current Position", getTurretRotation());
		Logger.recordOutput("Turret/Target Position", targetRotation);
		Logger.recordOutput("Turret/AtGoal", isAtGoal());
	}

	public Command trackTarget() {
		return run(() -> {
			var data = shotCalculator.getData();
			Logger.recordOutput("Turret/ShotData Exists", data != null);
			if (data != null) {
				Logger.recordOutput("Turret/ShotData Valid", data.isValid());
				Rotation2d desired = data.turretAngle();
				Logger.recordOutput("Turret/ShotCalc Angle", desired);
				if (data.isValid()) {
					setSetpoint(desired);
					Logger.recordOutput("Turret/ShotCalc Angle Goal", desired);
				}
			}
		});
	}

	private void configureTurret() {
		// Encoder conversion: motor rotations → radians
		turretConfig.encoder
				.positionConversionFactor(ShooterConstants.kTurretGearRatio * 2.0 * Math.PI)
				.velocityConversionFactor(ShooterConstants.kTurretGearRatio * 2.0 * Math.PI / 60.0);

		turretConfig
				.idleMode(IdleMode.kBrake)
				.inverted(true)
				.closedLoop
					.outputRange(-ShooterConstants.turretMaxPercentOutput.get(), ShooterConstants.turretMaxPercentOutput.get())
					.p(ShooterConstants.kPTurret.get())
					.i(ShooterConstants.kITurret.get())
					.d(ShooterConstants.kDTurret.get())
					.allowedClosedLoopError(ShooterConstants.turretTolerance.get(), ClosedLoopSlot.kSlot0)
				.feedForward
						.kS(ShooterConstants.kSTurret.get())
						.kV(ShooterConstants.kVTurret.get())
						.kA(ShooterConstants.kATurret.get());
		turretConfig.softLimit
				.forwardSoftLimit(ShooterConstants.turretMaxAngle.get())
				.reverseSoftLimit(ShooterConstants.turretMinAngle.get())
				.forwardSoftLimitEnabled(true)
				.reverseSoftLimitEnabled(true);

		turretConfig.smartCurrentLimit(20);

		// Use kNoPersistParameters to avoid slow flash writes
		turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	/**
	 * Updates the turret PID constants from NetworkTables if they've changed.
	 * This allows live tuning via AdvantageScope.
	 */
	private void updateTurretPID() {
		// Check if any values changed (using hashCode as ID)
		int id = this.hashCode();
		if (ShooterConstants.kPTurret.hasChanged(id) || ShooterConstants.kITurret.hasChanged(id) ||
				ShooterConstants.kDTurret.hasChanged(id) || ShooterConstants.turretTolerance.hasChanged(id) ||
				ShooterConstants.turretMaxPercentOutput.hasChanged(id)) {

			turretConfig.closedLoop
					.outputRange(-ShooterConstants.turretMaxPercentOutput.get(), ShooterConstants.turretMaxPercentOutput.get())
						.p(ShooterConstants.kPTurret.get())
						.i(ShooterConstants.kITurret.get())
						.d(ShooterConstants.kDTurret.get())
					.allowedClosedLoopError(ShooterConstants.turretTolerance.get(), ClosedLoopSlot.kSlot0)
					.feedForward
						.kS(ShooterConstants.kSTurret.get())
						.kV(ShooterConstants.kVTurret.get())
						.kA(ShooterConstants.kATurret.get());

			turretMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

			Logger.recordOutput("Turret/PID Status",
					String.format("Updated: P=%.3f I=%.3f D=%.3f MaxOut=%.2f",
							ShooterConstants.kPTurret.get(), ShooterConstants.kITurret.get(), ShooterConstants.kDTurret.get(), ShooterConstants.turretMaxPercentOutput.get()));
		}
	}
}
