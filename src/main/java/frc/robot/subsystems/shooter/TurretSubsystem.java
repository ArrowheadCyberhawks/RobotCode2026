package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private Rotation2d targetRotation = new Rotation2d();
  private final ShotCalculator shotCalculator = ShotCalculator.getInstance();
  private boolean trackingEnabled = true;

  public TurretSubsystem() {
    configureTurret();
    resetTurretEncoder();
  }

  public void moveTurretToRadians(double radians) {
    // Convenience wrapper for setSetpoint using radians
    setSetpoint(Rotation2d.fromRadians(radians));
  }

  public void moveTurretToDegrees(double degrees) {
    moveTurretToRadians(Math.toRadians(degrees));
  }

  public void setTurretVoltage(double volts) {
    turnMotor.setControl(new VoltageOut(volts));
  }

  public void stopTurret() {
    // Hold the current position as the new target and stop applying motion
    targetRotation = getTurretRotation();
    turnMotor.setControl(neutralOut);
  }

  public void resetTurretEncoder() {
    double mechanismRotations = turnMotor.getPosition().getValue().in(Rotations);
    double wrappedRot = mechanismRotations % 1.0;
    turnMotor.setPosition(wrappedRot);
  }

  public void manualResetTurretEncoder(double rotations) {
    // Position is in mechanism rotations with SensorToMechanismRatio configured
    turnMotor.setPosition(rotations);
  }

  /**
   * Set the turret target angle. This matches the TurretSubsystemNeo API but
   * uses TalonFX Motion Magic as the underlying controller.
   *
   * @param targetTurretAngle Desired turret angle. Zero is forward, CCW
   *     positive, CW negative.
   */
  public void setSetpoint(Rotation2d targetTurretAngle) {
    // Clamp to soft-limit range from ShooterConstants
    targetRotation = Rotation2d.fromRadians(
        Math.max(ShooterConstants.turretMinAngle.get(),
        Math.min(ShooterConstants.turretMaxAngle.get(),
        targetTurretAngle.getRadians())));

    // With SensorToMechanismRatio configured, positions are in mechanism
    // rotations. Convert radians to rotations for Motion Magic.
    double mechanismRotations = targetRotation.getRadians() / (2.0 * Math.PI);
    turnMotor.setControl(turretRequest.withPosition(mechanismRotations));
  }

  /** Returns the last requested turret angle. */
  public Rotation2d getSetpoint() {
    return targetRotation;
  }

  @Override
  public void periodic() {
    // Update PID values from LoggedTunableNumbers
    if (ShooterConstants.kPTurret.hasChanged(hashCode()) || 
      ShooterConstants.kITurret.hasChanged(hashCode()) || 
      ShooterConstants.kDTurret.hasChanged(hashCode()) ||
      ShooterConstants.kVTurret.hasChanged(hashCode()) ||
      ShooterConstants.kATurret.hasChanged(hashCode()) ||
      ShooterConstants.kSTurret.hasChanged(hashCode())) {
      // Reconfigure PID on the motor controller
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = ShooterConstants.kPTurret.get();
        slot0.kI = ShooterConstants.kITurret.get();
        slot0.kD = ShooterConstants.kDTurret.get();
        slot0.kV = ShooterConstants.kVTurret.get();
        slot0.kA = ShooterConstants.kATurret.get();
        slot0.kS = ShooterConstants.kSTurret.get();
        turnMotor.getConfigurator().apply(slot0);
    }
    
    // Log telemetry
    Logger.recordOutput("Turret/Current Position", getTurretRotation());
    Logger.recordOutput("Turret/Target Position", targetRotation);
    Logger.recordOutput("Turret/AtGoal", isAtGoal());
    Logger.recordOutput("Turret/is Flipping", isFlipping());
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
      var data = shotCalculator.getData();
      Logger.recordOutput("Turret/ShotData Exists", data != null);
      if (data != null) {
        Logger.recordOutput("Turret/ShotData Valid", data.isValid());
        Rotation2d desired = data.turretAngle();
        Logger.recordOutput("Turret/ShotCalc Angle", desired);
        if (data.isValid()) {
          setSetpoint(desired);
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
    double targetRadians = targetRotation.getRadians();

    // Handle angle wrapping for shortest distance
    double error = Math.abs(Rotation2d.fromRadians(currentRadians)
        .minus(Rotation2d.fromRadians(targetRadians))
        .getRadians());

    return error <= toleranceRadians;
  }

  /** Default goal check: within ~2 degrees of target. */
  public boolean isAtGoal() {
    return atGoal(Math.toRadians(2.0));
  }

  private void configureTurret() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.ClosedLoopGeneral.ContinuousWrap = false;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    //cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    cfg.Feedback.SensorToMechanismRatio = 1.0 / ShooterConstants.kTurretGearRatio;
    
    cfg.Slot0.kP = ShooterConstants.kPTurret.get();
    cfg.Slot0.kI = ShooterConstants.kITurret.get();
    cfg.Slot0.kD = ShooterConstants.kDTurret.get();
    cfg.Slot0.kV = ShooterConstants.kVTurret.get();
    cfg.Slot0.kA = ShooterConstants.kATurret.get();
    cfg.Slot0.kS = ShooterConstants.kSTurret.get();
    cfg.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kTurretCruiseRps;
    cfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.kTurretAccelRps2;

  	// Convert min/max radians to rotations of the turret mechanism
  	double maxMechanismRotations = ShooterConstants.turretMaxAngle.get() / (2.0 * Math.PI);
  	double minMechanismRotations = ShooterConstants.turretMinAngle.get() / (2.0 * Math.PI);
  	cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
  	cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxMechanismRotations;
  	cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minMechanismRotations;
    turnMotor.getConfigurator().apply(cfg);
  }

  /**
   * Returns true when the turret is making a large move ("flipping" across the
   * robot), mirroring the behavior of TurretSubsystemNeo.
   */

  public boolean isFlipping() {
    double currentPos = getTurretRotation().getRadians();
    double targetPos = targetRotation.getRadians();
    return Math.abs(currentPos - targetPos) > Math.PI / 4.0;
  }

}
