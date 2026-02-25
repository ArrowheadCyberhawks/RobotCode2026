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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
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
  private final PIDController pid;

  private boolean trackingEnabled = true;
  private double targetDegrees = ShooterConstants.HoodPosition.STOW.getDegrees();

  public HoodSubsystemNeo() {
    this(ShooterConstants.kHoodMotorId);
  }

  public HoodSubsystemNeo(int motorId) {
    hoodMotor = new SparkMax(motorId, MotorType.kBrushless);
    encoder = hoodMotor.getEncoder();
    pid = new PIDController(ShooterConstants.kPHood.get(), ShooterConstants.kIHood.get(), ShooterConstants.kDHood.get());
    pid.setTolerance(ShooterConstants.kHoodAllowedError);

    configureHood();
    resetHoodEncoderToDegrees(ShooterConstants.HoodPosition.STOW.getDegrees());
    pid.setSetpoint(Math.toRadians(ShooterConstants.HoodPosition.STOW.getDegrees()));
  

    Logger.recordOutput("Shooter/Hood Target", ShooterConstants.kShootHoodTarget);
  }

  private void configureHood() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    // convert motor rotations to hood radians for easier control/monitoring
    cfg.encoder.positionConversionFactor(ShooterConstants.kHoodGearRatio * 2.0 * Math.PI);
    cfg.idleMode(IdleMode.kBrake).inverted(false);
  //   cfg.closedLoop.positionWrappingEnabled(false)//.positionWrappingInputRange(-Math.PI, Math.PI)
  //       .p(ShooterConstants.kPHood.get())
  //       .i(ShooterConstants.kIHood.get())
  //       .d(ShooterConstants.kDHood.get())
  // .allowedClosedLoopError(ShooterConstants.kHoodAllowedError, ClosedLoopSlot.kSlot0);
    cfg.softLimit.forwardSoftLimit(Math.toRadians(ShooterConstants.kHoodMaxDegrees))
        .reverseSoftLimit(Math.toRadians(ShooterConstants.kHoodMinDegrees));

    cfg.smartCurrentLimit(20);

    // Use kNoPersistParameters to avoid slow flash writes that cause 6-second delays
    hoodMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
    targetDegrees = clipped; // Store target for atGoal() check
    // Convert degrees to radians since encoder is configured with radian conversion factor
    double radians = Math.toRadians(clipped);
    // SparkMax position controller expects the same units as the encoder conversion factor (radians)
    pid.setSetpoint(radians);
    Logger.recordOutput("Shooter/Hood Target", degrees);
  }

  public void stopHood() {
    hoodMotor.stopMotor();
  }

  /**
   * Check if the hood is at its goal position.
   * 
   * @param toleranceDegrees The tolerance in degrees
   * @return true if the hood is within tolerance of the target
   */
  public boolean atGoal(double toleranceDegrees) {
    double currentDegrees = getHoodAngle().in(Degrees);
    return Math.abs(currentDegrees - targetDegrees) <= toleranceDegrees;
  }

  /**
   * Check if the hood is at its goal position using default tolerance.
   * 
   * @return true if the hood is within 2 degrees of the target
   */
  public boolean isAtGoal() {
    return atGoal(2.0); // 2 degree default tolerance
  }

  public Angle getHoodAngle() {
    // Encoder returns radians due to conversion factor, convert to degrees
    return Radians.of(encoder.getPosition());
  }

  public Angle getHoodTargetAngle() {
    return Radians.of(pid.getSetpoint());
  }

  public void resetHoodEncoderToDegrees(double degrees) {
    // Convert degrees to radians for encoder position
    double radians = Math.toRadians(degrees);
    encoder.setPosition(radians);
  }

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
      // SparkMaxConfig cfg = new SparkMaxConfig();
      pid.setP(ShooterConstants.kPHood.get());
      pid.setI(ShooterConstants.kIHood.get());
      pid.setD(ShooterConstants.kDHood.get());
      // hoodMotor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    hoodMotor.setVoltage(Volts.of(pid.calculate(getHoodAngle().in(Radians))));
    Logger.recordOutput("Shooter/Hood Angle", getHoodAngle().in(Degrees));
    Logger.recordOutput("Shooter/Hood Current", hoodMotor.getOutputCurrent());
    // if (trackingEnabled) {
    //   var data = ShotCalculator.getInstance().getData();
    //   if (data != null && data.isValid()) {
    //     double hoodRad = data.hoodAngle();
    //     double hoodDeg = Math.toDegrees(hoodRad);
    //     moveHoodToDegrees(hoodDeg);
    //   }
    // }
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
