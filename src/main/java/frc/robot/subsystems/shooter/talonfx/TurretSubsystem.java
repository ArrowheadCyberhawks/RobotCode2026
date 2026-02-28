package frc.robot.subsystems.shooter.talonfx;

import com.ctre.phoenix6.configs.Slot0Configs;
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
import org.littletonrobotics.junction.Logger;

/**
 * Dedicated turret subsystem moved out of ShooterSubsystem for separation of
 * concerns.
 */
public class TurretSubsystem extends SubsystemBase {

  private final TalonFX turnMotor = new TalonFX(ShooterConstants.kTurretMotorId);
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
    // With SensorToMechanismRatio configured, positions are in mechanism rotations
    // Convert radians to rotations for the command
    double turretRotations = radians / (2.0 * Math.PI);
    turnMotor.setControl(turretRequest.withPosition(turretRotations));
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
    // With SensorToMechanismRatio configured, position is in mechanism rotations
    // Wrap to one rotation
    double mechanismRotations = turnMotor.getPosition().getValue().in(Rotations);
    double wrappedRot = mechanismRotations % 1.0;
    turnMotor.setPosition(wrappedRot);
  }

  public void manualResetTurretEncoder(double rotations) {
    // Position is in mechanism rotations with SensorToMechanismRatio configured
    turnMotor.setPosition(rotations);
  }

  @Override
  public void periodic() {
    // Update PID values from LoggedTunableNumbers
    if (ShooterConstants.kPTurret.hasChanged(hashCode()) || 
        ShooterConstants.kITurret.hasChanged(hashCode()) || 
        ShooterConstants.kDTurret.hasChanged(hashCode()) ||
        ShooterConstants.kSTurret.hasChanged(hashCode())) {
      // Reconfigure PID on the motor controller
      Slot0Configs slot0 = new Slot0Configs();
      slot0.kP = ShooterConstants.kPTurret.get();
      slot0.kI = ShooterConstants.kITurret.get();
      slot0.kD = ShooterConstants.kDTurret.get();
      slot0.kS = ShooterConstants.kSTurret.get();
      turnMotor.getConfigurator().apply(slot0);
    }
    
    // Log telemetry
    Logger.recordOutput("Turret/Current Position", getTurretRotation());
  }

  /**
   * Enable or disable automatic tracking using ShotCalculator
   */
  public void setTrackingEnabled(boolean enabled) {
    this.trackingEnabled = enabled;
  }

  /**
   * Query ShotCalculator for latest shot solution and command the turret to that position.
   */
  public Command trackTarget() {
    return run(() -> {
      var data = ShotCalculator.getInstance().getData();
      Logger.recordOutput("Turret/ShotData Exists", data != null);
      if (data != null) {
        Logger.recordOutput("Turret/ShotData Valid", data.isValid());
        Rotation2d desired = data.turretAngle();
        Logger.recordOutput("Turret/ShotCalc Angle", desired);
        if (data.isValid()) {
          moveTurretToRadians(desired.getRadians());
          Logger.recordOutput("Turret/ShotCalc Angle Goal", desired);
        }
      }
    });
  }

  public Rotation2d getTurretRotation() {
    try {
      // With SensorToMechanismRatio configured, getPosition returns mechanism rotations
      double mechanismRotations = turnMotor.getPosition().getValue().in(Rotations);
      return Rotation2d.fromRadians(mechanismRotations * 2.0 * Math.PI);
    } catch (Exception e) {
      return new Rotation2d();
    }
  }

  /**
   * Check if the turret is at its goal position.
   * 
   * @param toleranceRadians The tolerance in radians
   * @return true if the turret is within tolerance of the target
   */
  public boolean atGoal(double toleranceRadians) {
    double currentRadians = getTurretRotation().getRadians();
    double targetRotations = turretRequest.Position;
    double targetRadians = targetRotations * 2.0 * Math.PI;
    
    // Handle angle wrapping for shortest distance
    double error = Math.abs(Rotation2d.fromRadians(currentRadians)
        .minus(Rotation2d.fromRadians(targetRadians))
        .getRadians());
    
    return error <= toleranceRadians;
  }

  private void configureTurret() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.ClosedLoopGeneral.ContinuousWrap = true;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    // Configure ratio so ContinuousWrap works for the turret
    cfg.Feedback.SensorToMechanismRatio = 1.0 / ShooterConstants.kTurretGearRatio;
    
    cfg.Slot0.kP = ShooterConstants.kPTurret.get();
    cfg.Slot0.kI = ShooterConstants.kITurret.get();
    cfg.Slot0.kD = ShooterConstants.kDTurret.get();
    cfg.Slot0.kS = ShooterConstants.kSTurret.get();
    cfg.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kTurretCruiseRps;
    cfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.kTurretAccelRps2;
    turnMotor.getConfigurator().apply(cfg);
  }
}
