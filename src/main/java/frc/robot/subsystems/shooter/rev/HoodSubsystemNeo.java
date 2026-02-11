package frc.robot.subsystems.shooter.rev;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;

/**
 * Hood subsystem using REV NEO (CANSparkMax). Uses the built-in
 * position control on the SparkMax PID controller. The interface mirrors
 * the TalonFX-based hood so callers can swap implementations.
 */
public class HoodSubsystemNeo extends SubsystemBase {
  private final SparkMax hoodMotor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;

  private boolean trackingEnabled = false;

  public HoodSubsystemNeo() {
    this(ShooterConstants.kHoodMotorId);
  }

  public HoodSubsystemNeo(int motorId) {
    hoodMotor = new SparkMax(motorId, MotorType.kBrushless);
    encoder = hoodMotor.getEncoder();
    pid = hoodMotor.getClosedLoopController();

    configureHood();
    resetHoodEncoderToDegrees(ShooterConstants.HoodPosition.STOW.getDegrees());

    SmartDashboard.putNumber("Shooter/Hood Target", ShooterConstants.kShootHoodTarget);
  }

  private void configureHood() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    // convert motor rotations to hood radians for easier control/monitoring
    cfg.encoder.positionConversionFactor(ShooterConstants.kHoodGearRatio * 2.0 * Math.PI);
    cfg.idleMode(IdleMode.kBrake).inverted(true);
    cfg.closedLoop.positionWrappingEnabled(true).positionWrappingInputRange(-Math.PI, Math.PI)
        .p(ShooterConstants.kPHood).i(ShooterConstants.kIHood).d(ShooterConstants.kDHood)
  .allowedClosedLoopError(ShooterConstants.kHoodAllowedError, ClosedLoopSlot.kSlot0);
    cfg.softLimit.forwardSoftLimit(Math.toRadians(ShooterConstants.kHoodMaxDegrees))
        .reverseSoftLimit(Math.toRadians(ShooterConstants.kHoodMinDegrees));

    hoodMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setTrackingEnabled(boolean enabled) {
    this.trackingEnabled = enabled;
  }

  public void moveHoodTo(ShooterConstants.HoodPosition pos) {
    moveHoodToDegrees(pos.getDegrees());
  }

  public void moveHoodToDegrees(double degrees) {
    double clipped = Math.max(ShooterConstants.kHoodMinDegrees,
        Math.min(ShooterConstants.kHoodMaxDegrees, degrees));
    double motorRotations = degreesToMotorRotations(clipped);
    // SparkMax position controller expects rotations (motor rotations)
    pid.setReference(motorRotations, ControlType.kPosition);
  }

  public void stopHood() {
    hoodMotor.stopMotor();
  }

  public double getHoodDegrees() {
    double motorRot = encoder.getPosition();
    return motorRotationsToDegrees(motorRot);
  }

  public void resetHoodEncoderToDegrees(double degrees) {
    // SparkMax encoder doesn't allow direct setting of position via API in all
    // cases; instead we can set the position in the RelativeEncoder object.
    double motorRot = degreesToMotorRotations(degrees);
    encoder.setPosition(motorRot);
  }

  private double degreesToMotorRotations(double degrees) {
    return (degrees / 360.0) * ShooterConstants.kHoodGearRatio;
  }

  private double motorRotationsToDegrees(double motorRot) {
    return (motorRot / ShooterConstants.kHoodGearRatio) * 360.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Hood Degrees", getHoodDegrees());
    if (trackingEnabled) {
      var data = ShotCalculator.getInstance().getData();
      if (data != null && data.isValid()) {
        double hoodRad = data.hoodAngle();
        double hoodDeg = Math.toDegrees(hoodRad);
        moveHoodToDegrees(hoodDeg);
      }
    }
  }

  public Command trackTarget() {
    return run(() -> {
      var data = ShotCalculator.getInstance().getData();
      double hoodRad = data.hoodAngle();
      double hoodDeg = Math.toDegrees(hoodRad);
      moveHoodToDegrees(hoodDeg);
    });
  }
}
