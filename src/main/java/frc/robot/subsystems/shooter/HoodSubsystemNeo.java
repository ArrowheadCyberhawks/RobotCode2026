package frc.robot.subsystems.shooter;

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

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

// Using REV closed-loop controller (SparkClosedLoopController) instead of WPILib profiled PID
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Hood subsystem using REV NEO (CANSparkMax). Uses the built-in
 * position control on the SparkMax PID controller. The interface mirrors
 * the TalonFX-based hood so callers can swap implementations.
 */
public class HoodSubsystemNeo extends SubsystemBase {
  private final SparkMax hoodMotor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController hoodController;
  private SparkMaxConfig hoodConfig;
  private ShotCalculator shotCalculator = ShotCalculator.getInstance();

  private Rotation2d targetAngle = Rotation2d.fromDegrees(ShooterConstants.HoodPosition.STOW.getDegrees());

  public HoodSubsystemNeo() {
    this(ShooterConstants.kHoodMotorId);
  }

  public HoodSubsystemNeo(int motorId) {
    hoodMotor = new SparkMax(motorId, MotorType.kBrushless);
    encoder = hoodMotor.getEncoder();
      // Configure the Spark motor and closed-loop controller
      configureHood();
      resetHoodEncoder(Rotation2d.fromDegrees(ShooterConstants.HoodPosition.STOW.getDegrees()));

      // Obtain the Spark closed-loop controller after configuration
      hoodController = hoodMotor.getClosedLoopController();
      // Set initial target
      targetAngle = Rotation2d.fromDegrees(ShooterConstants.HoodPosition.STOW.getDegrees());
      hoodController.setReference(targetAngle.getRadians(), SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);

      Logger.recordOutput("Shooter/Hood Target", ShooterConstants.kShootHoodTarget);

    
  }

  private void configureHood() {
    SparkMaxConfig cfg = new SparkMaxConfig();
    // convert motor rotations to hood radians for easier control/monitoring
    cfg.encoder.positionConversionFactor(ShooterConstants.kHoodGearRatio * 2.0 * Math.PI);
    cfg.idleMode(IdleMode.kBrake).inverted(true);
  //   cfg.closedLoop.positionWrappingEnabled(false)//.positionWrappingInputRange(-Math.PI, Math.PI)
  //       .p(ShooterConstants.kPHood.get())
  //       .i(ShooterConstants.kIHood.get())
  //       .d(ShooterConstants.kDHood.get())
  // .allowedClosedLoopError(ShooterConstants.kHoodAllowedError, ClosedLoopSlot.kSlot0);
    cfg.softLimit.forwardSoftLimit(Math.toRadians(ShooterConstants.kHoodMaxDegrees))
        .reverseSoftLimit(Math.toRadians(ShooterConstants.kHoodMinDegrees));

    cfg.smartCurrentLimit(15, 10);

    // Use kNoPersistParameters to avoid slow flash writes that cause 6-second delays
    hoodMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Move the hood to a specific angle. The angle will be clipped to the configured min/max range.
   * @param angle Target angle as a Rotation2d. Zero is straight up, positive shoots the ball at a lower angle (raises the hood).
   */
  public void setSetpoint(Rotation2d angle) {
    double degrees = angle.getDegrees();
    double clipped = Math.max(ShooterConstants.kHoodMinDegrees,
        Math.min(ShooterConstants.kHoodMaxDegrees, degrees));
    targetAngle = Rotation2d.fromDegrees(clipped);
  // Command Spark closed-loop controller; encoder units are radians
  hoodController.setSetpoint(targetAngle.getRadians(), SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    Logger.recordOutput("Shooter/Hood Target", clipped);
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
    double currentDegrees = getHoodAngle().getDegrees();
    return Math.abs(currentDegrees - targetAngle.getDegrees()) <= toleranceDegrees;
  }

  /**
   * Check if the hood is at its goal position using default tolerance.
   * 
   * @return true if the hood is within 2 degrees of the target
   */
  public boolean isAtGoal() {
    return atGoal(2.0); // 2 degree default tolerance
  }

  public Rotation2d getHoodAngle() {
    // Encoder returns radians due to conversion factor
    return Rotation2d.fromRadians(encoder.getPosition());
  }

  public Rotation2d getSetpoint() {
    return targetAngle;
  }

  public void resetHoodEncoder(Rotation2d angle) {
    // Encoder uses radians due to conversion factor
    encoder.setPosition(angle.getRadians());
  }

  @Override
  public void periodic() {
    // Update Spark closed-loop PID constants if they've changed. Rebuild the closedLoop
    // config and re-apply with no-persist to avoid writing to flash on every change
    // while tuning.
    int id = hashCode();
    if (ShooterConstants.kPHood.hasChanged(id) || ShooterConstants.kIHood.hasChanged(id) || ShooterConstants.kDHood.hasChanged(id) ||
        ShooterConstants.kVHood.hasChanged(id) || ShooterConstants.kAHood.hasChanged(id) || ShooterConstants.kGHood.hasChanged(id)) {
      if (hoodConfig == null) {
        hoodConfig = new SparkMaxConfig();
      }
      hoodConfig.closedLoop
          .pid(ShooterConstants.kPHood.get(), ShooterConstants.kIHood.get(), ShooterConstants.kDHood.get())
          .feedForward
            .kV(ShooterConstants.kVHood.get());
      hoodConfig.closedLoop.allowedClosedLoopError(Math.toRadians(ShooterConstants.kHoodAllowedError), ClosedLoopSlot.kSlot0);
      hoodMotor.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    hoodController.setSetpoint(targetAngle.getRadians(), SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0);

    Logger.recordOutput("Shooter/Hood Angle", getHoodAngle().getDegrees());
    Logger.recordOutput("Shooter/Hood Current", hoodMotor.getOutputCurrent());
    Logger.recordOutput("Shooter/Hood AtGoal", isAtGoal());
  }

  public Command trackTarget() {
    return run(() -> 
      setSetpoint(shotCalculator.getData().hoodAngle())
    );
  }

  public Command down() {
    return runOnce(() -> {
      setSetpoint(ShooterConstants.HoodPosition.STOW.getRotation());
    });
  }
}
