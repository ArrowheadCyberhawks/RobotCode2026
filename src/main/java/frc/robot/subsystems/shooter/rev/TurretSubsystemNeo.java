package frc.robot.subsystems.shooter.rev;

import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.LoggedTunableNumber;

/**
 * Turret implementation using REV CANSparkMax (NEO). 
 * Uses REV onboard PID controller for position control.
 */
public class TurretSubsystemNeo extends SubsystemBase {
  private final SparkMax turnMotor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController turretController;
  
  private SparkMaxConfig turretConfig;
  
  // Tunable PID constants for Turret
  private final LoggedTunableNumber turretKP = new LoggedTunableNumber("Turret/kP", ShooterConstants.kPTurret.get());
  private final LoggedTunableNumber turretKI = new LoggedTunableNumber("Turret/kI", ShooterConstants.kITurret.get());
  private final LoggedTunableNumber turretKD = new LoggedTunableNumber("Turret/kD", ShooterConstants.kDTurret.get());
  private final LoggedTunableNumber turretTolerance = new LoggedTunableNumber("Turret/Tolerance", ShooterConstants.kTurretAllowedError);
  private final LoggedTunableNumber turretMaxPercentOutput = new LoggedTunableNumber("Turret/MaxPercentOutput", 1.0);

  private boolean trackingEnabled = true;
  private double targetRadians = 0.0;

  public TurretSubsystemNeo() {
    this(ShooterConstants.kTurretMotorId);
  }

  public TurretSubsystemNeo(int motorId) {
    turnMotor = new SparkMax(motorId, MotorType.kBrushless);
    encoder = turnMotor.getEncoder();
    turretController = turnMotor.getClosedLoopController();
    
    turretConfig = new SparkMaxConfig();

    configureTurret();
    resetTurretEncoder();
  }

  /**
   * Set target azimuth for the turret. The controller will rotate the turret to the specified angle.
   * @param targetTurretAngle Target angle in radians
   */
  public void setTurretTarget(Angle targetTurretAngle) {
    turretController.setReference(targetTurretAngle.in(Radians), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void moveTurretToRadians(double radians) {
    targetRadians = radians; // Store target for atGoal() check
    turretController.setReference(radians, ControlType.kPosition, ClosedLoopSlot.kSlot0);
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

  /**
   * Check if the turret is at its goal position.
   * 
   * @param toleranceRadians The tolerance in radians
   * @return true if the turret is within tolerance of the target
   */
  public boolean atGoal(double toleranceRadians) {
    double currentRadians = getTurretRotation().getRadians();
    
    // Handle angle wrapping for shortest distance
    double error = Math.abs(Rotation2d.fromRadians(currentRadians)
        .minus(Rotation2d.fromRadians(targetRadians))
        .getRadians());
    
    return error <= toleranceRadians;
  }

  /**
   * Check if the turret is at its goal position using default tolerance.
   * 
   * @return true if the turret is within 3 degrees (~0.052 radians) of the target
   */
  public boolean isAtGoal() {
    return atGoal(Math.toRadians(3.0)); // 3 degree default tolerance
  }

  public void resetTurretEncoder() {
    // Get current position in radians
    double currentRadians = encoder.getPosition();
    // Wrap to [-π, π] range to match ShotCalculator
    double wrappedRadians = Math.atan2(Math.sin(currentRadians), Math.cos(currentRadians));
    encoder.setPosition(wrappedRadians);
  }

  public void manualResetTurretEncoder(double rotations) {
    encoder.setPosition(rotations);
  }

  @Override
  public void periodic() {
    // Update PID constants if they've changed in NetworkTables
    updateTurretPID();
    
    // Periodically wrap encoder to [-π, π] to match ShotCalculator
    // TODO - check if you actually need this with rev
    double currentRadians = getTurretRotation().getRadians();
    if (Math.abs(currentRadians) > 10.0 * Math.PI) {
      double wrappedRadians = Math.atan2(Math.sin(currentRadians), Math.cos(currentRadians));
      encoder.setPosition(wrappedRadians);
    }
    
    // Log telemetry
    SmartDashboard.putNumber("Turret/Current Position (deg)", getTurretRotation().getDegrees());
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
        SmartDashboard.putNumber("Turret/ShotCalc Angle (rad)", desired.getRadians());
        if (data.isValid()) {
          moveTurretToRadians(desired.getRadians());
          SmartDashboard.putNumber("Turret/ShotCalc Angle Goal", desired.getDegrees());
        }
      }
    });
  }

  public Rotation2d getTurretRotation() {
    try {
      return Rotation2d.fromRadians(encoder.getPosition());
    } catch (Exception e) {
      return new Rotation2d();
    }
  }

  private void configureTurret() {
    // Encoder conversion: motor rotations → radians
    turretConfig.encoder.positionConversionFactor(ShooterConstants.kTurretGearRatio * 2.0 * Math.PI);
    turretConfig.encoder.velocityConversionFactor(ShooterConstants.kTurretGearRatio * 2.0 * Math.PI / 60.0);
    
    turretConfig.idleMode(IdleMode.kBrake)
      .inverted(true)
      .closedLoop
        .outputRange(-turretMaxPercentOutput.get(), turretMaxPercentOutput.get())
        .p(turretKP.get())
        .i(turretKI.get())
        .d(turretKD.get())
        .allowedClosedLoopError(turretTolerance.get(), ClosedLoopSlot.kSlot0);
  
    // Use kNoPersistParameters to avoid slow flash writes
    turnMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Updates the turret PID constants from NetworkTables if they've changed.
   * This allows live tuning via AdvantageScope or SmartDashboard.
   */
  private void updateTurretPID() {
    // Check if any values changed (using hashCode as ID)
    int id = this.hashCode();
    if (turretKP.hasChanged(id) || turretKI.hasChanged(id) || 
        turretKD.hasChanged(id) || turretTolerance.hasChanged(id) ||
        turretMaxPercentOutput.hasChanged(id)) {
      
      turretConfig.closedLoop
          .outputRange(-turretMaxPercentOutput.get(), turretMaxPercentOutput.get())
          .p(turretKP.get())
          .i(turretKI.get())
          .d(turretKD.get())
          .allowedClosedLoopError(turretTolerance.get(), ClosedLoopSlot.kSlot0);
      
      turnMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      
      SmartDashboard.putString("Turret/PID Status", 
          String.format("Updated: P=%.3f I=%.3f D=%.3f MaxOut=%.2f", 
              turretKP.get(), turretKI.get(), turretKD.get(), turretMaxPercentOutput.get()));
    }
  }
}
