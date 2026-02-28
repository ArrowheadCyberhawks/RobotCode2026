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
 * Flywheel implementation using REV CANSparkMax (NEO) hardware with velocity control.
 */
public class FlywheelSubsystemNeo extends SubsystemBase {
  // private final SparkMax leader;
  // //private final SparkMax follower;
  // private final RelativeEncoder encoder;
  // private final SparkClosedLoopController pid;

  // private double velocitySetpointRadsPerSec = 0.0;
  // private boolean atGoal = false;

  // // Slew rate limiter to smooth setpoint changes (rad/s per second)
  // private final SlewRateLimiter setpointLimiter = new SlewRateLimiter(500.0);

  // private static final LoggedTunableNumber velocityTolerance =
  //     new LoggedTunableNumber("Flywheel/VelocityTolerance", 20.0);

  // @AutoLogOutput private int launchCount = 0;
  // private boolean lastAtGoal = false;

  // public FlywheelSubsystemNeo(int leaderId, int followerId) {
  // leader = new SparkMax(leaderId, MotorType.kBrushless);
  //   //follower = new SparkMax(followerId, MotorType.kBrushless);

  //   // Configure the motors using SparkMaxConfig (preferred API in vendordeps)
  //   SparkMaxConfig cfg = new SparkMaxConfig();
  //   // Configure idle mode and closed-loop gains via SparkMaxConfig. Some
  //   // vendordeps versions expose slightly different configuration chains; we
  //   // set the commonly available fields here.
  //   cfg.idleMode(IdleMode.kCoast)
  //     .closedLoop
  //       .p(ShooterConstants.kPFlywheel.get())
  //       .i(ShooterConstants.kIFlywheel.get())
  //       .d(ShooterConstants.kDFlywheel.get())
  //     .feedForward
  //       .kV(ShooterConstants.kVFlywheel.get())
  //       .kS(ShooterConstants.kSFlywheel.get());
  //   cfg.inverted(true);

  //   leader.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  //   //follower.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  //   encoder = leader.getEncoder();
  //   pid = leader.getClosedLoopController();
  // }

  // /** Convenience ctor using constants */
  // public FlywheelSubsystemNeo() {
  //   this(ShooterConstants.kFlywheelMotorId, ShooterConstants.kShootMotorId);
  // }

  // // configurePID removed: configuration is applied via SparkMaxConfig in the
  // // constructor because vendordeps API surface varies between versions.

  // @Override
  // public void periodic() {
  //   // Update PID constants if they changed
  //   if (ShooterConstants.kPFlywheel.hasChanged(hashCode()) 
  //       || ShooterConstants.kIFlywheel.hasChanged(hashCode())
  //       || ShooterConstants.kDFlywheel.hasChanged(hashCode())
  //       || ShooterConstants.kVFlywheel.hasChanged(hashCode())
  //       || ShooterConstants.kSFlywheel.hasChanged(hashCode())) {
  //     SparkMaxConfig cfg = new SparkMaxConfig();
  //     cfg.idleMode(IdleMode.kCoast)
  //       .closedLoop
  //         .p(ShooterConstants.kPFlywheel.get())
  //         .i(ShooterConstants.kIFlywheel.get())
  //         .d(ShooterConstants.kDFlywheel.get())
  //       .feedForward
  //         .kV(ShooterConstants.kVFlywheel.get())
  //         .kS(ShooterConstants.kSFlywheel.get());
  //     cfg.inverted(true);
  //     leader.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  //   }

  //   double rpm = encoder.getVelocity(); // RPM
  //   double velocityRadPerSec = rpm * 2.0 * Math.PI / 60.0;
  //   Logger.recordOutput("Flywheel/Velocity", velocityRadPerSec);

  //   if (velocitySetpointRadsPerSec == 0.0) {
  //     leader.stopMotor();
  //     atGoal = false;
  //     return;
  //   }

  //   boolean inTolerance = Math.abs(velocityRadPerSec - velocitySetpointRadsPerSec) <= velocityTolerance.get();
  //   atGoal = inTolerance;

  //   // Always use velocity control
  //   // Convert rad/s target to RPM for SparkMax
  //   double targetRpm = velocitySetpointRadsPerSec * 60.0 / (2.0 * Math.PI);
  //   pid.setReference(targetRpm, ControlType.kVelocity);

  //   Logger.recordOutput("Flywheel/Setpoint", velocitySetpointRadsPerSec);
  //   Logger.recordOutput("Flywheel/AtGoal", atGoal);
  //   Logger.recordOutput("Flywheel/Current", leader.getOutputCurrent());
  // }

  // private void runVelocity(double velocityRadsPerSec) {
  //   // Apply slew rate limiting to smooth setpoint changes
  //   velocitySetpointRadsPerSec = setpointLimiter.calculate(velocityRadsPerSec);
  // }

  // private void stop() {
  //   velocitySetpointRadsPerSec = 0.0;
  //   setpointLimiter.reset(0.0);
  //   atGoal = false;
  // }

  // public double getVelocity() {
  //   double rpm = encoder.getVelocity();
  //   return rpm * 2.0 * Math.PI / 60.0;
  // }

  // public boolean isAtGoal() {
  //   return atGoal;
  // }

  // public long getLaunchCount() {
  //   return launchCount;
  // }

  // public Command trackTarget() {
  //   return runEnd(() -> runVelocity(ShotCalculator.getInstance().getData().flywheelSpeed()), this::stop);
  // }

  // public Command runFixedCommand(DoubleSupplier velocity) {
  //   return runEnd(() -> runVelocity(velocity.getAsDouble()), this::stop);
  // }

  // public Command stopCommand() {
  //   return runOnce(this::stop);
  // }
}
