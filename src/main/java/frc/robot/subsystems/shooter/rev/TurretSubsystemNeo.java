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
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;

/**
 * Turret implementation using REV CANSparkMax (NEO). Uses position control
 * on the SparkMax PID controller. Mirrors the TalonFX-based Turret API.
 */
public class TurretSubsystemNeo extends SubsystemBase {
  private final SparkMax turnMotor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  private boolean trackingEnabled = true;

  public TurretSubsystemNeo() {
    this(ShooterConstants.kTurnMotorId);
  }

  public TurretSubsystemNeo(int motorId) {
    turnMotor = new SparkMax(motorId, MotorType.kBrushless);
    encoder = turnMotor.getEncoder();
    pid = turnMotor.getClosedLoopController();

    configureTurret();
    resetTurretEncoder();
  }

  public void moveTurretToRadians(double radians) {
    double currentRad = getTurretRotation().getRadians();
    double rawDiff = radians - currentRad;
    double delta = Math.atan2(Math.sin(rawDiff), Math.cos(rawDiff));
    double shortestRad = currentRad + delta;
    double turretRotations = shortestRad / (2.0 * Math.PI);
    double motorRotations = turretRotations * ShooterConstants.kTurretGearRatio;
    pid.setReference(motorRotations, ControlType.kPosition);
  }

  public void moveTurretToDegrees(double degrees) {
    moveTurretToRadians(Math.toRadians(degrees));
  }

  public void setTurretVoltage(double volts) {
    turnMotor.setVoltage(volts);
  }

  public void stopTurret() {
    turnMotor.stopMotor();
  }

  public void resetTurretEncoder() {
    double rot = encoder.getPosition();
    encoder.setPosition(rot % 1.0);
  }

  @Override
  public void periodic() {
    // optional tracking occurs in periodic in TalonFX version; here we leave
    // tracking to explicit commands or the trackTarget() Command below.
  }

  public void setTrackingEnabled(boolean enabled) {
    this.trackingEnabled = enabled;
  }

  public Command trackTarget() {
    return run(() -> {
      var data = ShotCalculator.getInstance().getData();
      if (data != null && data.isValid()) {
        Rotation2d desired = data.turretAngle();
        moveTurretToRadians(desired.getRadians());
      }
    });
  }

  public Rotation2d getTurretRotation() {
    try {
      double rotations = encoder.getPosition();
      double turretRotations = rotations / ShooterConstants.kTurretGearRatio;
      return Rotation2d.fromRadians(turretRotations * 2.0 * Math.PI);
    } catch (Exception e) {
      return new Rotation2d();
    }
  }

  private void configureTurret() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.encoder.positionConversionFactor(ShooterConstants.kTurretGearRatio * 2.0 * Math.PI);
    cfg.idleMode(IdleMode.kBrake).inverted(true).closedLoop
        .outputRange(-ShooterConstants.kMaxTurretVolts, ShooterConstants.kMaxTurretVolts)
        .p(ShooterConstants.kPTurret)
        .i(ShooterConstants.kITurret)
        .d(ShooterConstants.kDTurret)
        .allowedClosedLoopError(ShooterConstants.kTurretAllowedError, ClosedLoopSlot.kSlot0);

    turnMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
