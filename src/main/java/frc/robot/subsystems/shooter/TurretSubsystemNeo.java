package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
/**
 * Turret implementation using REV CANSparkMax (NEO).
 * Uses REV onboard PID controller for position control.
 */
public class TurretSubsystemNeo extends SubsystemBase {
	private final SparkMax turretMotor;
	private final RelativeEncoder encoder;
	private final PIDController pidController;
	private final ProfiledPIDController profiledPIDController;
	private SimpleMotorFeedforward feedforward;
	
	private ShotCalculator shotCalculator = ShotCalculator.getInstance();

	private SparkMaxConfig turretConfig;	
	private Rotation2d targetRotation = new Rotation2d();

	public TurretSubsystemNeo() {
		this(ShooterConstants.kTurretMotorId);
	}

	public TurretSubsystemNeo(int motorId) {
		turretMotor = new SparkMax(motorId, MotorType.kBrushless);
		encoder = turretMotor.getEncoder();

		pidController = new PIDController(
			ShooterConstants.kPTurret.get(),
			ShooterConstants.kITurret.get(),
			ShooterConstants.kDTurret.get()
		);
		
		//unused
		profiledPIDController = new ProfiledPIDController(
			ShooterConstants.kPTurret.get(),
			ShooterConstants.kITurret.get(),
			ShooterConstants.kDTurret.get(),
			new TrapezoidProfile.Constraints(
				ShooterConstants.kMaxTurretVelocity.get(),
				ShooterConstants.kMaxTurretAcceleration.get()
			)
		);

		feedforward = new SimpleMotorFeedforward(
			ShooterConstants.kSTurret.get(),
			ShooterConstants.kVTurret.get(),
			ShooterConstants.kATurret.get()
		);

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

	public void setTurretVoltage(Voltage volts) {
		turretMotor.setVoltage(volts);
	}

	public void stopTurret() {
		targetRotation = getTurretRotation(); // Update target to current position to prevent jumps when resuming control
		turretMotor.setVoltage(0.0);
	}

	/**
	 * Check if the turret is at its goal position.
	 * 
	 * @param toleranceRadians The tolerance in radians
	 * @return true if the turret is within tolerance of the target
	 */
	public boolean atGoal(double toleranceRadians) {
		double currentRadians = getTurretRotation().getRadians();
		double error = Math.abs(targetRotation.getRadians() - currentRadians);
		return error <= toleranceRadians;
	}

	/**
	 * Check if the turret is at its goal position using default tolerance.
	 * 
	 * @return true if the turret is within 2 degrees of the target
	 */
	public boolean isAtGoal() {
		return atGoal(Math.toRadians(2.0)); // 2 degree default tolerance
	}

	public void manualResetTurretEncoder(double rotations) {
		encoder.setPosition(rotations);
	}

	@Override
	public void periodic() {
		// Update PID constants if they've changed in NetworkTables
		updateTurretPID();
		
		double currentPos = getTurretRotation().getRadians();
		double targetPos = targetRotation.getRadians();

		double pidOut = pidController.calculate(currentPos, targetPos);
		
		// Use ProfiledPIDController's current setpoint velocity for feedforward
		double setpointVelocity = shotCalculator.getData().turretVelocity();
		double ffOut = isAtGoal() ? 0.0 : feedforward.calculate(setpointVelocity);
		//ffOut = 0.0;

		// Apply voltage clamp
		double maxVolts = ShooterConstants.turretMaxPercentOutput.get() * 12.0;
		double totalOut = MathUtil.clamp(pidOut + ffOut, -maxVolts, maxVolts);

		turretMotor.setVoltage(Volts.of(totalOut));

		// Log telemetry
		Logger.recordOutput("Turret/Current Position", getTurretRotation());
		Logger.recordOutput("Turret/Target Position", targetRotation);
		Logger.recordOutput("Turret/Target Velocity", setpointVelocity);
		Logger.recordOutput("Turret/PID Output", pidOut);
		Logger.recordOutput("Turret/Feedforward Output", ffOut);
		Logger.recordOutput("Turret/Current", turretMotor.getOutputCurrent());
		Logger.recordOutput("Turret/AtGoal", isAtGoal());
		Logger.recordOutput("Turret/is Flipping", isFlipping());
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
				.inverted(true);
		turretConfig.softLimit
				.forwardSoftLimit(ShooterConstants.turretMaxAngle.get())
				.reverseSoftLimit(ShooterConstants.turretMinAngle.get())
				.forwardSoftLimitEnabled(true)
				.reverseSoftLimitEnabled(true);

		turretConfig.smartCurrentLimit(40);

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

			pidController.setPID(
				ShooterConstants.kPTurret.get(),
				ShooterConstants.kITurret.get(),
				ShooterConstants.kDTurret.get()
			);

			feedforward = new SimpleMotorFeedforward(
				ShooterConstants.kSTurret.get(),
				ShooterConstants.kVTurret.get(),
				ShooterConstants.kATurret.get()
			);

			// We still log the change
			Logger.recordOutput("Turret/PID Status",
					String.format("Updated: P=%.3f I=%.3f D=%.3f MaxOut=%.2f",
							ShooterConstants.kPTurret.get(), ShooterConstants.kITurret.get(), ShooterConstants.kDTurret.get(), ShooterConstants.turretMaxPercentOutput.get()));
		}
	}

	public boolean isFlipping() {
		double currentPos = getTurretRotation().getRadians();
		double targetPos = targetRotation.getRadians();

    	return Math.abs(currentPos - targetPos) > Math.PI / 4.00;
  	}
}
