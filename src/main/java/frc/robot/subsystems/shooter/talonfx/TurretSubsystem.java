package frc.robot.subsystems.shooter.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.ShotCalculator.ShotData;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Rotations;

/**
 * Dedicated turret subsystem moved out of ShooterSubsystem for separation of
 * concerns.
 */
public class TurretSubsystem extends SubsystemBase {

  private final TalonFX turnMotor = new TalonFX(ShooterConstants.kTurnMotorId);
  private final MotionMagicVoltage turretRequest = new MotionMagicVoltage(0);
  private final VoltageOut neutralOut = new VoltageOut(0);

  // When enabled, the turret will use ShotCalculator.getInstance() to compute
  // the desired turret azimuth and drive there each periodic loop.
  private boolean trackingEnabled = true;

  public TurretSubsystem() {
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
    turnMotor.setControl(turretRequest.withPosition(motorRotations));
  }

  public void moveTurretToDegrees(double degrees) {
    moveTurretToRadians(Math.toRadians(degrees));
  }

  public void setTurretVoltage(double volts) {
    turnMotor.setControl(new VoltageOut(volts));
  }

  public void stopTurret() {
    turnMotor.setControl(neutralOut);
  }

  public void resetTurretEncoder() {
    double rot = turnMotor.getPosition().getValue().in(Rotations);
    turnMotor.setPosition(rot % 1.0);
  }

  @Override
  public void periodic() {

  }

  /**
   * Enable or disable automatic tracking using ShotCalculator
   */
  public void setTrackingEnabled(boolean enabled) {
    this.trackingEnabled = enabled;
  }

  /**
   * Query ShotCalculator for latest shot solution and command the turret tothat posotion.
   */
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
      double rotations = turnMotor.getPosition().getValue().in(Rotations);
      double turretRotations = rotations / ShooterConstants.kTurretGearRatio;
      return Rotation2d.fromRadians(turretRotations * 2.0 * Math.PI);
    } catch (Exception e) {
      return new Rotation2d();
    }
  }

  private void configureTurret() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.ClosedLoopGeneral.ContinuousWrap = true;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Slot0.kP = ShooterConstants.kPTurret;
    cfg.Slot0.kI = ShooterConstants.kITurret;
    cfg.Slot0.kD = ShooterConstants.kDTurret;
    cfg.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kTurretCruiseRps;
    cfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.kTurretAccelRps2;
    turnMotor.getConfigurator().apply(cfg);
  }
}
