package frc.robot.subsystems.shooter.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
// Hood can optionally follow a target computed by ShotCalculator when
// trackingEnabled is set.
import frc.robot.subsystems.shooter.ShooterConstants.HoodPosition;
import frc.robot.subsystems.shooter.ShotCalculator.ShotData;

/**
 * Hood subsystem extracted from ShooterSubsystem. Responsible for configuring
 * and commanding the hood TalonFX using Motion Magic positions.
 */
public class HoodSubsystem extends SubsystemBase {

  private final TalonFX hood = new TalonFX(ShooterConstants.kHoodMotorId);
  private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);
  private final NeutralOut neutralOut = new NeutralOut();

  public HoodSubsystem() {
    configureHood();
    // Zero to known STOW position on construction (caller can re-zero later).
    resetHoodEncoderToDegrees(ShooterConstants.HoodPosition.STOW.getDegrees());

    SmartDashboard.putNumber("Shooter/Hood Target", ShooterConstants.kShootHoodTarget);
  }

  private boolean trackingEnabled = false;

  /**
   * Enable or disable automatic hood tracking driven by ShotCalculator.
   */
  public void setTrackingEnabled(boolean enabled) {
    this.trackingEnabled = enabled;
  }

  private void configureHood() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Slot0.kP = ShooterConstants.kPHood;
    cfg.Slot0.kI = ShooterConstants.kIHood;
    cfg.Slot0.kD = ShooterConstants.kDHood;
    cfg.Slot0.kV = ShooterConstants.kVHood;
    cfg.Slot0.kA = ShooterConstants.kAHood;
    cfg.Slot0.kG = ShooterConstants.kGHood;
    cfg.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kHoodCruiseRps;
    cfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.kHoodAccelRps2;
    hood.getConfigurator().apply(cfg);
  }

  public void moveHoodTo(ShooterConstants.HoodPosition pos) {
    hood.setControl(hoodRequest.withPosition(degreesToMotorRotations(pos.getDegrees())));
  }

  public void moveHoodToDegrees(double degrees) {
    double clipped = Math.max(ShooterConstants.kHoodMinDegrees,
        Math.min(ShooterConstants.kHoodMaxDegrees, degrees));
    hood.setControl(hoodRequest.withPosition(degreesToMotorRotations(clipped)));
  }

  public void stopHood() {
    hood.setControl(neutralOut);
  }

  public double getHoodDegrees() {
    return motorRotationsToDegrees(hood.getPosition().getValueAsDouble());
  }

  public void resetHoodEncoderToDegrees(double degrees) {
    hood.setPosition(degreesToMotorRotations(degrees));
  }

  // Conversions
  private double degreesToMotorRotations(double degrees) {
    return (degrees / 360.0) * ShooterConstants.kHoodGearRatio;
  }

  private double motorRotationsToDegrees(double motorRot) {
    return (motorRot / ShooterConstants.kHoodGearRatio) * 360.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Hood Degrees", getHoodDegrees());
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


