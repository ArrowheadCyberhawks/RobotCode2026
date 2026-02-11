package frc.robot.subsystems.shooter.rev;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Flywheel implementation using REV CANSparkMax (NEO) hardware.
 */
public class FlywheelSubsystemNeo extends SubsystemBase {
  private final SparkMax leader;
  private final SparkMax follower;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  private double velocitySetpointRadsPerSec = 0.0;
  private boolean torqueControl = false;
  private boolean atGoal = false;

  private static final LoggedTunableNumber torqueCurrentControlTolerance =
      new LoggedTunableNumber("Flywheel/TorqueCurrentControlTolerance", 20.0);
  private static final LoggedTunableNumber torqueCurrentControlDebounce =
      new LoggedTunableNumber("Flywheel/TorqueCurrentControlDebounce", 0.025);
  private static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Flywheel/AtGoalDebounce", 0.2);

  private Debouncer torqueCurrentDebouncer = new Debouncer(torqueCurrentControlDebounce.get(), DebounceType.kFalling);
  private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);

  private boolean lastTorqueCurrentControl = false;

  @AutoLogOutput private int launchCount = 0;

  public enum ControlMode {
    BANG_BANG,
    VELOCITY
  }

  private ControlMode controlMode = ControlMode.BANG_BANG;

  public FlywheelSubsystemNeo(int leaderId, int followerId) {
  leader = new SparkMax(leaderId, MotorType.kBrushless);
    follower = new SparkMax(followerId, MotorType.kBrushless);

    // Configure the motors using SparkMaxConfig (preferred API in vendordeps)
    SparkMaxConfig cfg = new SparkMaxConfig();
    // Configure idle mode and closed-loop gains via SparkMaxConfig. Some
    // vendordeps versions expose slightly different configuration chains; we
    // set the commonly available fields here.
    cfg.idleMode(IdleMode.kCoast).closedLoop.p(ShooterConstants.kPFlywheel).i(ShooterConstants.kIFlywheel)
        .d(ShooterConstants.kDFlywheel);

    leader.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = leader.getEncoder();
    pid = leader.getClosedLoopController();
  }

  /** Convenience ctor using constants */
  public FlywheelSubsystemNeo() {
    this(ShooterConstants.kFlywheelMotorId, ShooterConstants.kShootMotorId);
  }

  // configurePID removed: configuration is applied via SparkMaxConfig in the
  // constructor because vendordeps API surface varies between versions.

  @Override
  public void periodic() {
    double rpm = encoder.getVelocity(); // RPM
    double velocityRadPerSec = rpm * 2.0 * Math.PI / 60.0;
    Logger.recordOutput("Flywheel/Velocity", velocityRadPerSec);

    // update debouncers if tunables changed (best-effort)
    if (torqueCurrentControlDebounce.hasChanged(hashCode())) {
      torqueCurrentDebouncer = new Debouncer(torqueCurrentControlDebounce.get(), DebounceType.kFalling);
    }
    if (atGoalDebounce.hasChanged(hashCode())) {
      atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);
    }

    if (velocitySetpointRadsPerSec == 0.0) {
      leader.stopMotor();
      atGoal = false;
      torqueControl = false;
      return;
    }

    boolean inTolerance = Math.abs(velocityRadPerSec - velocitySetpointRadsPerSec) <= torqueCurrentControlTolerance.get();

    torqueControl = torqueCurrentDebouncer.calculate(inTolerance);
    atGoal = atGoalDebouncer.calculate(inTolerance);

    if (!torqueControl && lastTorqueCurrentControl) {
      launchCount++;
    }
    lastTorqueCurrentControl = torqueControl;

    // Apply control: if torque-like control is requested, use PID velocity control
    // (REV has limited current control; for now, use velocity control; otherwise use full power)
    if (torqueControl || controlMode == ControlMode.VELOCITY) {
  // Convert rad/s target to RPM for SparkMax
  double targetRpm = velocitySetpointRadsPerSec * 60.0 / (2.0 * Math.PI);
  pid.setReference(targetRpm, ControlType.kVelocity);
    } else {
      // Bang-bang (simple) control: set motor to full power in sign of target
      leader.set(Math.copySign(1.0, velocitySetpointRadsPerSec));
    }

    Logger.recordOutput("Flywheel/Setpoint", velocitySetpointRadsPerSec);
    Logger.recordOutput("Flywheel/TorqueCurrentControl", torqueControl);
  }

  private void runVelocity(double velocityRadsPerSec) {
    velocitySetpointRadsPerSec = velocityRadsPerSec;
  }

  private void stop() {
    velocitySetpointRadsPerSec = 0.0;
    atGoal = false;
  }

  public double getVelocity() {
    double rpm = encoder.getVelocity();
    return rpm * 2.0 * Math.PI / 60.0;
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
