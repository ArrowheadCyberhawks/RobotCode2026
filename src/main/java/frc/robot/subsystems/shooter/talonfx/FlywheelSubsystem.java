package frc.robot.subsystems.shooter.talonfx;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


public class FlywheelSubsystem extends SubsystemBase {

  private final TalonFX leader;
  private final TalonFX follower;

  // Velocity closed-loop control request using torque current FOC (reused to avoid object allocation)
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  private AngularVelocity velocitySetpoint = RadiansPerSecond.of(0.0);
  private boolean atGoal = false;

  // Slew rate limiter to smooth setpoint changes (rad/s per second)
  private final SlewRateLimiter setpointLimiter = new SlewRateLimiter(500.0);

  // Tuning published in NetworkTables via LoggedTunableNumber (used for runtime tolerances)
  private static final LoggedTunableNumber velocityTolerance =
      new LoggedTunableNumber("Flywheel/VelocityTolerance", 5.0);

  public FlywheelSubsystem(int leaderId, int followerId) {
    leader = new TalonFX(leaderId);
    follower = new TalonFX(followerId);

    configureFlywheel();
    
    // Configure follower to mirror leader (opposed direction for typical flywheels)
    leader.setControl(velocityRequest);
    follower.setControl(new Follower(leaderId, MotorAlignmentValue.Opposed));

  }

  public FlywheelSubsystem() {
    this(ShooterConstants.kFlywheelMotorId, ShooterConstants.kFlywheelFollowerMotorId);
  }

  private void configureFlywheel() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    
    // Set neutral mode to coast for flywheel
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Configure Slot0 closed-loop gains (used by VelocityTorqueCurrentFOC)
    // Note: VelocityTorqueCurrentFOC uses kP, kI, kD, kV, kS like VelocityDutyCycle
    cfg.Slot0.kP = ShooterConstants.kPFlywheel.get();
    cfg.Slot0.kI = ShooterConstants.kIFlywheel.get();
    cfg.Slot0.kD = ShooterConstants.kDFlywheel.get();
    cfg.Slot0.kV = ShooterConstants.kVFlywheel.get();
    cfg.Slot0.kS = ShooterConstants.kSFlywheel.get();

    // Configure current limits for FOC
    cfg.CurrentLimits.SupplyCurrentLimit = 40.0; // Amps
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    follower.getConfigurator().apply(cfg);
    leader.getConfigurator().apply(cfg);
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
      cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      
      // Configure current limits for FOC
      cfg.CurrentLimits.SupplyCurrentLimit = 40.0; // Amps
      cfg.CurrentLimits.SupplyCurrentLimitEnable = false;
      follower.getConfigurator().apply(cfg);
      leader.getConfigurator().apply(cfg);
    }

    // Get current velocity in rotations per second, convert to rad/s for internal use
    AngularVelocity currentVelocity = RotationsPerSecond.of(leader.getVelocity().getValueAsDouble());

    Logger.recordOutput("Flywheel/Setpoint", velocitySetpoint);
    Logger.recordOutput("Flywheel/Velocity", currentVelocity);
    Logger.recordOutput("Flywheel/AtGoal", atGoal);
    Logger.recordOutput("Flywheel/Error", leader.getClosedLoopError().getValueAsDouble());
    Logger.recordOutput("Flywheel/Current", leader.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Flywheel/TorqueCurrent", leader.getTorqueCurrent().getValueAsDouble());

    if (velocitySetpoint.isEquivalent(RadiansPerSecond.zero())) {
      leader.stopMotor();
      atGoal = false;
      return;
    }

    boolean inTolerance = currentVelocity.isNear(velocitySetpoint, RadiansPerSecond.of(velocityTolerance.get()));
    atGoal = inTolerance;

    // Use VelocityTorqueCurrentFOC closed-loop control
    velocityRequest.Velocity = velocitySetpoint.in(RotationsPerSecond);
    velocityRequest.FeedForward = 2.90;
    velocityRequest.EnableFOC = false;
    leader.setControl(velocityRequest);

    Logger.recordOutput("Flywheel/Setpoint", velocitySetpoint);
    Logger.recordOutput("Flywheel/AtGoal", atGoal);
    Logger.recordOutput("Flywheel/Current", leader.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Flywheel/TorqueCurrent", leader.getTorqueCurrent().getValueAsDouble());
  }

  // setpoint runner used by commands and direct callers
  public void runVelocity(double velocityRadsPerSec) {
    // Apply slew rate limiting to smooth setpoint changes
    velocitySetpoint = RadiansPerSecond.of(setpointLimiter.calculate(velocityRadsPerSec));
  }

  public void stop() {
    velocitySetpoint = RadiansPerSecond.zero();
    setpointLimiter.reset(0.0);
    atGoal = false;
  }

  public double getVelocity() {
    // Return velocity in rad/s (convert from TalonFX's rps)
    return leader.getVelocity().getValueAsDouble() * 2.0 * Math.PI;
  }

  public boolean isAtGoal() {
    return atGoal;
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

  public Command diagnosePhase() {
    return run(() -> {
        leader.set(0.1);
        double velocity = leader.getVelocity().getValueAsDouble();
        Logger.recordOutput("Flywheel/DiagnoseVelocity", velocity);
        if (velocity < 0) {
          Logger.recordOutput("Flywheel/Sensor", "INVERTED");
        } else {
          Logger.recordOutput("Flywheel/Sensor", "CORRECT");
        }
      });



    
  }
}

