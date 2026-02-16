package frc.robot.subsystems.shooter.talonfx;

import com.ctre.phoenix6.configs.Slot0Configs;
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
    cfg.Slot0.kP = ShooterConstants.kPHood.get();
    cfg.Slot0.kI = ShooterConstants.kIHood.get();
    cfg.Slot0.kD = ShooterConstants.kDHood.get();
    cfg.Slot0.kV = ShooterConstants.kVHood.get();
    cfg.Slot0.kA = ShooterConstants.kAHood.get();
    cfg.Slot0.kG = ShooterConstants.kGHood.get();
    cfg.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kHoodCruiseRps;
    cfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.kHoodAccelRps2;
    
    // Soft limits in motor rotations
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degreesToMotorRotations(ShooterConstants.kHoodMaxDegrees);
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degreesToMotorRotations(ShooterConstants.kHoodMinDegrees);
    
    hood.getConfigurator().apply(cfg);
  }

  public void moveHoodTo(ShooterConstants.HoodPosition pos) {
    hood.setControl(hoodRequest.withPosition(degreesToMotorRotations(pos.getDegrees())));
  }

  public void moveHoodToDegrees(double degrees) {
    double clipped = Math.max(ShooterConstants.kHoodMinDegrees,
        Math.min(ShooterConstants.kHoodMaxDegrees, degrees));
    hood.setControl(hoodRequest.withPosition(degreesToMotorRotations(clipped)));
    SmartDashboard.putNumber("Shooter/Hood Target", degrees);
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
    // Update PID values from LoggedTunableNumbers
    if (ShooterConstants.kPHood.hasChanged(hashCode()) || 
        ShooterConstants.kIHood.hasChanged(hashCode()) || 
        ShooterConstants.kDHood.hasChanged(hashCode()) ||
        ShooterConstants.kVHood.hasChanged(hashCode()) ||
        ShooterConstants.kAHood.hasChanged(hashCode()) ||
        ShooterConstants.kGHood.hasChanged(hashCode())) {
      // Reconfigure PID on the motor controller
      Slot0Configs slot0 = new Slot0Configs();
      slot0.kP = ShooterConstants.kPHood.get();
      slot0.kI = ShooterConstants.kIHood.get();
      slot0.kD = ShooterConstants.kDHood.get();
      slot0.kV = ShooterConstants.kVHood.get();
      slot0.kA = ShooterConstants.kAHood.get();
      slot0.kG = ShooterConstants.kGHood.get();
      hood.getConfigurator().apply(slot0);
    }
    
    SmartDashboard.putNumber("Shooter/Hood Degrees", getHoodDegrees());
    
  //   // Track target if enabled -> I turned this into a command
  //   if (trackingEnabled) {
  //     var data = ShotCalculator.getInstance().getData();
  //     if (data != null && data.isValid()) {
  //       double hoodRad = data.hoodAngle();
  //       double hoodDeg = Math.toDegrees(hoodRad);
  //       moveHoodToDegrees(hoodDeg);
  //     }
  //   }
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


