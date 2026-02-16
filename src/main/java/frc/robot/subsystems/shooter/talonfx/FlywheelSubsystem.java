package frc.robot.subsystems.shooter.talonfx;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * TalonFX-based flywheel using closed-loop velocity control (VelocityDutyCycle).
 * Mirrors the Neo API (runFixedCommand/trackTarget/stopCommand) but uses the TalonFX
 * onboard closed-loop velocity slot configured at construction time.
 */
public class FlywheelSubsystem extends SubsystemBase {

  private final TalonFX leader;
  private final TalonFX follower;

  // Velocity closed-loop control request (reused to avoid object allocation)
  private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0.0);

  private double velocitySetpointRadsPerSec = 0.0;
  private boolean atGoal = false;

  // Slew rate limiter to smooth setpoint changes (rad/s per second)
  private final SlewRateLimiter setpointLimiter = new SlewRateLimiter(500.0);

  // Tuning published in NetworkTables via LoggedTunableNumber (used for runtime tolerances)
  private static final LoggedTunableNumber velocityTolerance =
      new LoggedTunableNumber("Flywheel/VelocityTolerance", 20.0);

  @AutoLogOutput private int launchCount = 0;
  private boolean lastAtGoal = false;

  public FlywheelSubsystem(int leaderId, int followerId) {
    leader = new TalonFX(leaderId);
    follower = new TalonFX(followerId);

    // Configure follower to mirror leader (opposed direction for typical flywheels)
    follower.setControl(new Follower(leaderId, MotorAlignmentValue.Opposed));

    configureFlywheel();
  }

  public FlywheelSubsystem() {
    this(ShooterConstants.kFlywheelMotorId, ShooterConstants.kShootMotorId);
  }

  private void configureFlywheel() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    // Set neutral mode to coast for flywheel
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Configure Slot0 closed-loop gains (used by VelocityDutyCycle)
    cfg.Slot0.kP = ShooterConstants.kPFlywheel.get();
    cfg.Slot0.kI = ShooterConstants.kIFlywheel.get();
    cfg.Slot0.kD = ShooterConstants.kDFlywheel.get();
    cfg.Slot0.kV = ShooterConstants.kVFlywheel.get();
    cfg.Slot0.kS = ShooterConstants.kSFlywheel.get();

    leader.getConfigurator().apply(cfg);
    follower.getConfigurator().apply(cfg);
  }

  @Override
  public void periodic() {
    // Update PID constants if they changed
    if (ShooterConstants.kPFlywheel.hasChanged(hashCode()) 
        || ShooterConstants.kIFlywheel.hasChanged(hashCode())
        || ShooterConstants.kDFlywheel.hasChanged(hashCode())
        || ShooterConstants.kVFlywheel.hasChanged(hashCode())
        || ShooterConstants.kSFlywheel.hasChanged(hashCode())) {
      TalonFXConfiguration cfg = new TalonFXConfiguration();
      cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      cfg.Slot0.kP = ShooterConstants.kPFlywheel.get();
      cfg.Slot0.kI = ShooterConstants.kIFlywheel.get();
      cfg.Slot0.kD = ShooterConstants.kDFlywheel.get();
      cfg.Slot0.kV = ShooterConstants.kVFlywheel.get();
      cfg.Slot0.kS = ShooterConstants.kSFlywheel.get();
      leader.getConfigurator().apply(cfg);
      follower.getConfigurator().apply(cfg);
    }

    double velocityRadPerSec = leader.getVelocity().getValueAsDouble();
    Logger.recordOutput("Flywheel/Velocity", velocityRadPerSec);

    if (velocitySetpointRadsPerSec == 0.0) {
      leader.stopMotor();
      atGoal = false;
      return;
    }

    boolean inTolerance = Math.abs(velocityRadPerSec - velocitySetpointRadsPerSec) <= velocityTolerance.get();
    atGoal = inTolerance;

    // Use closed-loop velocity control
    // Convert rad/s to rotations/s (Phoenix 6 uses rotations per second for velocity)
    double velocityRps = velocitySetpointRadsPerSec / (2.0 * Math.PI);
    velocityRequest.Velocity = velocityRps;
    leader.setControl(velocityRequest);

    Logger.recordOutput("Flywheel/Setpoint", velocitySetpointRadsPerSec);
    Logger.recordOutput("Flywheel/AtGoal", atGoal);
    Logger.recordOutput("Flywheel/Current", leader.getSupplyCurrent().getValueAsDouble());
  }

  // setpoint runner used by commands
  private void runVelocity(double velocityRadsPerSec) {
    // Apply slew rate limiting to smooth setpoint changes
    velocitySetpointRadsPerSec = setpointLimiter.calculate(velocityRadsPerSec);
  }

  private void stop() {
    velocitySetpointRadsPerSec = 0.0;
    setpointLimiter.reset(0.0);
    atGoal = false;
  }

  public double getVelocity() {
    return leader.getVelocity().getValueAsDouble();
  }

  public boolean isAtGoal() {
    return atGoal;
  }

  public long getLaunchCount() {
    return launchCount;
  }

  public Command trackTarget() {
    return runEnd(() -> runVelocity(ShotCalculator.getInstance().getData().flywheelSpeed()), this::stop);
  }

  public Command runFixedCommand(DoubleSupplier velocity) {
    return runEnd(() -> runVelocity(velocity.getAsDouble()), this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}

