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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

  // Store setpoint internally in rotations per second to match TalonFX units
  private double velocitySetpointRps = 0.0;
  private boolean atGoal = false;

  // Slew rate limiter to smooth setpoint changes (rotations/s per second)
  // Maybe lower this because this is basically instantanious
  private final SlewRateLimiter setpointLimiter = new SlewRateLimiter(160.0);

  // Tuning published in NetworkTables via LoggedTunableNumber (used for runtime tolerances)
  private static final LoggedTunableNumber velocityTolerance =
      new LoggedTunableNumber("Flywheel/VelocityTolerance", 5.0);

  public FlywheelSubsystem(int leaderId, int followerId) {
    leader = new TalonFX(leaderId);
    follower = new TalonFX(followerId);

    configureFlywheel();

    velocityRequest.EnableFOC = true;
    
    // Configure follower to mirror leader (opposed direction for typical flywheels)
    //leader.setControl(velocityRequest);
    leader.setControl(velocityTorqueCurrentRequest);
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
      cfg.CurrentLimits.StatorCurrentLimit = 40.0;
      cfg.CurrentLimits.StatorCurrentLimitEnable = true;
      follower.getConfigurator().apply(cfg);
      leader.getConfigurator().apply(cfg);
    }

    // Get current velocity in rotations per second, convert to rad/s for internal use

  AngularVelocity currentVelocity = RotationsPerSecond.of(leader.getVelocity().getValueAsDouble());

  Logger.recordOutput("Flywheel/SetpointRPS", velocitySetpointRps);
  Logger.recordOutput("Flywheel/VelocityRPS", currentVelocity.in(RotationsPerSecond));
    Logger.recordOutput("Flywheel/AtGoal", isAtGoal());
    Logger.recordOutput("Flywheel/ClosedLoopErrorRPS", leader.getClosedLoopError().getValueAsDouble());
    Logger.recordOutput("Flywheel/Current", leader.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Flywheel/TorqueCurrent", leader.getTorqueCurrent().getValueAsDouble());

    // Use VelocityVoltage closed-loop control even when the setpoint is zero so the error converges to 0
    velocityRequest.Velocity = velocitySetpointRps;
    velocityTorqueCurrentRequest.Velocity = velocitySetpointRps;
    leader.setControl(velocityRequest);
    //leader.setControl(velocityTorqueCurrentRequest);
    // Realized velocitytorquecontrol probably doesn't actually work bc I forgot to set it to use that in periodic,
    // here it is above if you want to test it
  }

  // setpoint runner used by commands and direct callers
  public void runVelocity(AngularVelocity velocity) {
    double targetRps = velocity.in(RotationsPerSecond);
    double minRps = ShooterConstants.kFlywheelMinVel.in(RotationsPerSecond);
    double maxRps = ShooterConstants.kFlywheelMaxVel.in(RotationsPerSecond);

    // Apply slew rate limiting and clamp in TalonFX-native units (rotations per second)
    velocitySetpointRps = MathUtil.clamp(targetRps, minRps, maxRps); //TODO: why doesn't the slew rate limiter work?

    // The slew rate limiter didn't work because it never calculated velocitySetpointRps using the rate limiter :D
    velocitySetpointRps = setpointLimiter.calculate(velocitySetpointRps);
    System.out.println(getVelocitySetpoint().in(RotationsPerSecond));
  }

  public void stop() {
    velocitySetpointRps = 0.0;
    setpointLimiter.reset(0.0);
    atGoal = false;
  }

  public AngularVelocity getVelocity() {
    // Return velocity in rad/s (convert from TalonFX's rps)
    return RotationsPerSecond.of(leader.getVelocity().getValueAsDouble());
  }

  public AngularVelocity getVelocitySetpoint() {
    return RotationsPerSecond.of(velocitySetpointRps);
  }

  public boolean isAtGoal() {
    // Compute error in TalonFX-native units (RPS)
    double measuredRps = leader.getVelocity().getValueAsDouble();
    double errorRps = velocitySetpointRps - measuredRps;
    double toleranceRps = velocityTolerance.get() / (2.0 * Math.PI); // convert rad/s -> rps

    return Math.abs(velocitySetpointRps) > 1e-4 && Math.abs(errorRps) <= toleranceRps;
  }

  public Command trackTarget() {
    //TODO: get rid of radians per second completely and just run everything in rotations per second since TalonFX uses that natively and it's less confusing to convert back and forth all the time
    return runEnd(() -> runVelocity(RadiansPerSecond.of(ShotCalculator.getInstance().getData().flywheelSpeed())), this::stop);
  }

  public Command runFixedCommand(Supplier<AngularVelocity> velocity) {
    return runEnd(() -> runVelocity(velocity.get()), this::stop);
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

