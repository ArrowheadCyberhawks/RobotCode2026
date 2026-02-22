package frc.robot.subsystems.shooter.talonfx;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;


public class FlywheelSubsystem extends SubsystemBase {

  private final TalonFX leader;
  private final TalonFX follower;

  // Simple duty cycle (percent output) control request for testing
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

  private double dutyCycleSetpoint = 0.0;
  private boolean atGoal = false;

  // Tuning published in NetworkTables via LoggedTunableNumber (used for runtime tolerances)
  private static final LoggedTunableNumber velocityTolerance =
      new LoggedTunableNumber("Flywheel/VelocityTolerance", 5.0);

  public FlywheelSubsystem(int leaderId, int followerId) {
    leader = new TalonFX(leaderId);
    follower = new TalonFX(followerId);

    // Configure follower to mirror leader (opposed direction for typical flywheels)
    follower.setControl(new Follower(leaderId, MotorAlignmentValue.Opposed));

    configureFlywheel();
  }

  public FlywheelSubsystem() {
    this(ShooterConstants.kFlywheelMotorId, ShooterConstants.kFlywheelFollowerMotorId);
  }

  private void configureFlywheel() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    // Set neutral mode to coast for flywheel
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Apply base config to follower first (without inversion)
    follower.getConfigurator().apply(cfg);
    // Then apply to leader with inversion
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leader.getConfigurator().apply(cfg);
  }

  @Override
  public void periodic() {
    // Get current velocity in rotations per second
    double currentRps = leader.getVelocity().getValueAsDouble();

    Logger.recordOutput("Flywheel/DutyCycleSetpoint", dutyCycleSetpoint);
    Logger.recordOutput("Flywheel/VelocityRPS", currentRps);
    Logger.recordOutput("Flywheel/VelocityRadPerSec", currentRps * 2.0 * Math.PI);
    Logger.recordOutput("Flywheel/AtGoal", atGoal);
    Logger.recordOutput("Flywheel/Current", leader.getSupplyCurrent().getValueAsDouble());

    if (dutyCycleSetpoint == 0.0) {
      leader.stopMotor();
      atGoal = false;
      return;
    }

    // For duty cycle mode, just check if motor is spinning
    atGoal = Math.abs(currentRps) > 1.0;

    // Apply simple duty cycle control
    dutyCycleRequest.Output = dutyCycleSetpoint;
    leader.setControl(dutyCycleRequest);
  }

  /**
   * Set the flywheel duty cycle output.
   *
   * @param dutyCycle duty cycle from -1.0 to 1.0
   */
  public void runVelocity(double dutyCycle) {
    dutyCycleSetpoint = dutyCycle;
  }

  public void stop() {
    dutyCycleSetpoint = 0.0;
    atGoal = false;
  }

  public double getVelocity() {
    // Return velocity in rad/s (convert from TalonFX's rps)
    return leader.getVelocity().getValueAsDouble() * 2.0 * Math.PI;
  }

  public boolean isAtGoal() {
    return atGoal;
  }

  public Command trackTarget() {
    return runEnd(() -> runVelocity(ShotCalculator.getInstance().getData().flywheelSpeed()), this::stop);
  }

  public Command runFixedCommand(DoubleSupplier velocity) {
    return runEnd(() -> runVelocity(velocity.getAsDouble()), this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}

