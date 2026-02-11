package frc.robot.subsystems.shooter.talonfx;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


public class FlywheelSubsystem extends SubsystemBase {

  private final TalonFX flywheel;
  private final TalonFX flywheelFollower;

  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0);

  private double velocitySetpointRadsPerSec = 0.0;
  private boolean torqueCurrentControl = false;
  private boolean atGoal = false;


  private static final LoggedTunableNumber torqueCurrentControlTolerance =
      new LoggedTunableNumber("Flywheel/TorqueCurrentControlTolerance", 20.0);
  private static final LoggedTunableNumber torqueCurrentControlDebounce =
      new LoggedTunableNumber("Flywheel/TorqueCurrentControlDebounce", 0.025);
  private static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Flywheel/AtGoalDebounce", 0.2);

  private Debouncer torqueCurrentDebouncer = new Debouncer(torqueCurrentControlDebounce.get(), DebounceType.kFalling);
  private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);

  private boolean lastTorqueCurrentControl = false;

  @AutoLogOutput private int launchCount = 0;

  public enum ControlMode {
    BANG_BANG,
    VELOCITY
  }

  //perhaps for any testing? we could implement both and see which works best
  private ControlMode controlMode = ControlMode.BANG_BANG;

  public FlywheelSubsystem(int motorId, int followerId) {
    flywheel = new TalonFX(motorId);
    flywheelFollower = new TalonFX(followerId);
    flywheelFollower.setControl(new Follower(motorId, MotorAlignmentValue.Opposed));
    // Apply PID/neutral-mode configuration copied from ShooterSubsystem
    configureFlywheel();
  }

  /** Configure the flywheel TalonFXs (PID/neutral mode). */
  private void configureFlywheel() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.Slot0.kP = ShooterConstants.kPFlywheel;
    cfg.Slot0.kI = ShooterConstants.kIFlywheel;
    cfg.Slot0.kD = ShooterConstants.kDFlywheel;
    cfg.Slot0.kV = ShooterConstants.kVFlywheel;
    // Apply to leader and follower
    try {
      flywheel.getConfigurator().apply(cfg);
      flywheelFollower.getConfigurator().apply(cfg);
    } catch (Exception e) {
      //avoid issues in constructor
    }
  }

  public FlywheelSubsystem() {
    this(ShooterConstants.kFlywheelMotorId, ShooterConstants.kShootMotorId);
  }

@Override
public void periodic() {
    //read sensors / state
    double velocity = flywheel.getVelocity().getValueAsDouble();
    Logger.recordOutput("Flywheel/Velocity", velocity);

    //update debouncers
    if (torqueCurrentControlDebounce.hasChanged(hashCode())) {
        torqueCurrentDebouncer = new Debouncer(torqueCurrentControlDebounce.get(), DebounceType.kFalling);
    }
    if (atGoalDebounce.hasChanged(hashCode())) {
        atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);
    }

    //skip if velocity is zero
    if (velocitySetpointRadsPerSec == 0.0) {
        flywheel.stopMotor();
        atGoal = false;
        torqueCurrentControl = false;
        return;
    }

    //closed-loop/bang-bang control
    boolean inTolerance = Math.abs(velocity - velocitySetpointRadsPerSec) <= torqueCurrentControlTolerance.get();

    torqueCurrentControl = torqueCurrentDebouncer.calculate(inTolerance);
    atGoal = atGoalDebouncer.calculate(inTolerance);

    //count number of launches
    if (!torqueCurrentControl && lastTorqueCurrentControl) {
        launchCount++;
    }
    lastTorqueCurrentControl = torqueCurrentControl;

    // Apply control
    if (torqueCurrentControl) {
        flywheel.setControl(torqueCurrentRequest.withOutput(velocitySetpointRadsPerSec));
    } else {
        flywheel.setControl(dutyCycleRequest.withOutput(Math.copySign(1.0, velocitySetpointRadsPerSec)));
    }

    Logger.recordOutput("Flywheel/Setpoint", velocitySetpointRadsPerSec);
    Logger.recordOutput("Flywheel/TorqueCurrentControl", torqueCurrentControl);
}


  private void runVelocity(double velocityRadsPerSec) {
    velocitySetpointRadsPerSec = velocityRadsPerSec;
  }

  private void stop() {
    velocitySetpointRadsPerSec = 0.0;
    atGoal = false;
  }

  public double getVelocity() {
    return flywheel.getVelocity().getValueAsDouble();
  }

  public boolean isAtGoal() {
    return atGoal;
  }

  public long getLaunchCount() {
    return launchCount;
  }

  public Command trackTarget() {
    return runEnd(
        () -> runVelocity(ShotCalculator.getInstance().getData().flywheelSpeed()),
        this::stop);
  }

  public Command runFixedCommand(DoubleSupplier velocity) {
    return runEnd(() -> runVelocity(velocity.getAsDouble()), this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}

