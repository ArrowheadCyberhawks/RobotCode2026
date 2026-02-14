package frc.robot.subsystems.shooter.rev;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;

/**
 * Turret implementation using REV CANSparkMax (NEO). 
 * Uses WPILib ProfiledPIDController (RobotRIO-side) for smooth motion profiling.
 */
public class TurretSubsystemNeo extends SubsystemBase {
  private final SparkMax turnMotor;
  private final RelativeEncoder encoder;
  
  // WPILib ProfiledPIDController for RobotRIO-side control
  private final ProfiledPIDController turretController;

  private boolean trackingEnabled = true;
  private double desiredAngleDegrees = 0.0;

  public TurretSubsystemNeo() {
    this(ShooterConstants.kTurnMotorId);
  }

  public TurretSubsystemNeo(int motorId) {
    turnMotor = new SparkMax(motorId, MotorType.kBrushless);
    encoder = turnMotor.getEncoder();
    
    // Create WPILib ProfiledPIDController with trapezoidal motion profile (RobotRIO-side)
    // Convert cruise velocity and acceleration from motor rotations/sec to turret radians/sec
    double maxVelocityRadPerSec = ShooterConstants.kTurretCruiseRps / ShooterConstants.kTurretGearRatio * 2.0 * Math.PI;
    double maxAccelRadPerSec2 = ShooterConstants.kTurretAccelRps2 / ShooterConstants.kTurretGearRatio * 2.0 * Math.PI;
    
    turretController = new ProfiledPIDController(
        ShooterConstants.kPTurret.get(),
        ShooterConstants.kITurret.get(),
        ShooterConstants.kDTurret.get(),
        new TrapezoidProfile.Constraints(maxVelocityRadPerSec, maxAccelRadPerSec2)
    );
    turretController.setTolerance(ShooterConstants.kTurretAllowedError);
    turretController.enableContinuousInput(0, 2.0 * Math.PI); // Wrap around for continuous rotation [0, 2π]

    configureTurret();
    resetTurretEncoder();
  }

  public void moveTurretToRadians(double radians) {
    desiredAngleDegrees = Math.toDegrees(radians);
    turretController.setGoal(radians);
  }

  public void moveTurretToDegrees(double degrees) {
    moveTurretToRadians(Math.toRadians(degrees));
  }

  public void setTurretVoltage(double volts) {
    turnMotor.setVoltage(volts);
  }

  public void stopTurret() {
    turnMotor.stopMotor();
    turretController.reset(getTurretRotation().getRadians());
  }

  public void resetTurretEncoder() {
    // Get current position in radians
    double currentRadians = encoder.getPosition();
    // Wrap to [0, 2π] range
    double wrappedRadians = currentRadians % (2.0 * Math.PI);
    if (wrappedRadians < 0) {
      wrappedRadians += 2.0 * Math.PI;
    }
    encoder.setPosition(wrappedRadians);
    turretController.reset(wrappedRadians);
  }

  @Override
  public void periodic() {
    // Update PID values from LoggedTunableNumbers
    if (ShooterConstants.kPTurret.hasChanged(hashCode()) || 
        ShooterConstants.kITurret.hasChanged(hashCode()) || 
        ShooterConstants.kDTurret.hasChanged(hashCode())) {
      turretController.setPID(
          ShooterConstants.kPTurret.get(),
          ShooterConstants.kITurret.get(),
          ShooterConstants.kDTurret.get()
      );
    }
    
    // Get current turret position in radians
    double currentRadians = getTurretRotation().getRadians();
    
    // Periodically wrap encoder to prevent unbounded growth
    // Only wrap if we're far outside the normal range
    if (currentRadians < 0 || currentRadians > 2.0 * Math.PI) {
      double wrappedRadians = currentRadians % (2.0 * Math.PI);
      if (wrappedRadians < 0) {
        wrappedRadians += 2.0 * Math.PI;
      }
      encoder.setPosition(wrappedRadians);
      currentRadians = wrappedRadians;
    }
    
    // Get goal from profiled controller
    double goalRadians = turretController.getGoal().position;
    
    // Calculate PID output using ProfiledPIDController
    double pidOutput = turretController.calculate(currentRadians);
    
    // Add static friction feedforward (kS)
    double kS = ShooterConstants.kSTurret.get();
    double feedforward = 0.0;
    if (Math.abs(pidOutput) > 0.01) { // Only apply if there's meaningful output
      feedforward = Math.copySign(kS, pidOutput);
    }
    
    // Combine PID and feedforward
    double totalOutput = pidOutput + feedforward;
    
    // Clamp output to max voltage
    double voltage = Math.max(-ShooterConstants.kMaxTurretVolts, 
                             Math.min(ShooterConstants.kMaxTurretVolts, totalOutput));
    
    // Apply voltage to motor
    turnMotor.setVoltage(voltage);
    
    // Log telemetry
    double currentDegrees = Math.toDegrees(currentRadians);
    double goalDegrees = Math.toDegrees(goalRadians);
    SmartDashboard.putNumber("Turret/Current Position (deg)", currentDegrees);
    SmartDashboard.putNumber("Turret/Goal Position (deg)", goalDegrees);
    SmartDashboard.putNumber("Turret/Desired Position (deg)", desiredAngleDegrees);
    SmartDashboard.putNumber("Turret/Error (deg)", goalDegrees - currentDegrees);
    SmartDashboard.putNumber("Turret/PID Output", pidOutput);
    SmartDashboard.putNumber("Turret/Voltage Output", voltage);
    SmartDashboard.putBoolean("Turret/At Goal", turretController.atGoal());
  }

  public void setTrackingEnabled(boolean enabled) {
    this.trackingEnabled = enabled;
  }

  public Command trackTarget() {
    return run(() -> {
      var data = ShotCalculator.getInstance().getData();
      SmartDashboard.putBoolean("Turret/ShotData Exists", data != null);
      if (data != null) {
        SmartDashboard.putBoolean("Turret/ShotData Valid", data.isValid());
        Rotation2d desired = data.turretAngle();
        SmartDashboard.putNumber("Turret/ShotCalc Angle (deg)", desired.getDegrees());
        if (data.isValid()) {
          moveTurretToRadians(desired.getRadians());
        }
      }
    });
  }

  public Rotation2d getTurretRotation() {
      return Rotation2d.fromRadians(encoder.getPosition());
  }

  private void configureTurret() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    // Encoder conversion: motor rotations → radians
    cfg.encoder.positionConversionFactor(ShooterConstants.kTurretGearRatio * 2.0 * Math.PI);
    cfg.encoder.velocityConversionFactor(ShooterConstants.kTurretGearRatio * 2.0 * Math.PI / 60.0);
    cfg.idleMode(IdleMode.kBrake).inverted(true);

    // No PID on motor controller - using WPILib ProfiledPIDController
    // Use kNoPersistParameters to avoid slow flash writes that cause 6-second delays
    turnMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
