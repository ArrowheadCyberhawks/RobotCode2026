package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  
  public enum ShooterState {
    IDLE,
    STRAIGHT,
    AIM,
    PASS,
    TRENCH,
    MANUAL
  }

  private final FlywheelSubsystem flywheel;
  private final HoodSubsystemNeo hood;
  private final TurretSubsystemNeo turret;

  private ShooterState currentState = ShooterState.IDLE;
  private ShooterState desiredState = ShooterState.IDLE;

  public ShooterSubsystem(
      FlywheelSubsystem flywheel,
      HoodSubsystemNeo hood,
      TurretSubsystemNeo turret) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    
  }

  @Override
  public void periodic() {
    updateState();
    executeState();

    Logger.recordOutput("Shooter/State", currentState.name());
    Logger.recordOutput("Shooter/AllAtGoal", areAllSubsystemsAtGoal());
  }

  public void requestState(ShooterState state) {
    desiredState = state;
  }

  public ShooterState getState() {
    return currentState;
  }

  public void stop() {
    requestState(ShooterState.IDLE);
  }

  public boolean areAllSubsystemsAtGoal() {
    return flywheel.isAtGoal() && hood.isAtGoal() && turret.isAtGoal();
  }

  public HoodSubsystemNeo getHoodSubsystem() {
    return hood;
  }

  public TurretSubsystemNeo getTurretSubsystem() {
      return turret;
  }

  public FlywheelSubsystem getFlywheelSubsystem() {
      return flywheel;
  }

  /**
   * Returns true whenever all subsystems are at their goal.
   */
  public boolean isReady() {
    return areAllSubsystemsAtGoal();
  }

  private void updateState() {
    if (currentState != desiredState) {
      currentState = desiredState;
    }
  }

  private void executeState() {
    switch (currentState) {

      case IDLE:
        flywheel.stop();
        hood.setSetpoint(ShooterConstants.HoodPosition.STOW.getRotation());
        turret.stopTurret();
        break;

      case STRAIGHT:
        turret.setSetpoint(new Rotation2d());

        var straightData = ShotCalculator.getInstance().getData();
        if (straightData != null && straightData.isValid()) {
          flywheel.setSetpoint(RadiansPerSecond.of(straightData.flywheelSpeed()));
          hood.setSetpoint(straightData.hoodAngle());
        }
        break;

      case AIM:
        var aimData = ShotCalculator.getInstance().getData();
        if (aimData != null && aimData.isValid()) {
          flywheel.setSetpoint(RotationsPerSecond.of(aimData.flywheelSpeed()));
          hood.setSetpoint(aimData.hoodAngle());
          turret.setSetpoint(aimData.turretAngle());
        }
        break;
      
      case PASS:
        var shotData = ShotCalculator.getInstance().getData();
        if (shotData != null && shotData.isValid()) {
          flywheel.setSetpoint(RotationsPerSecond.of(shotData.flywheelSpeed()));
          hood.setSetpoint(Rotation2d.fromDegrees(40));
          turret.setSetpoint(shotData.turretAngle());
        }
        break;

      case TRENCH:
        hood.setSetpoint(ShooterConstants.HoodPosition.STOW.getRotation());
        break;

      case MANUAL:
        break;

    }
  }

  // Simplification so you don't have to use run()

  /**
   * Command to set the shooter to the IDLE state, which stops all subsystems and holds them in place.
   * @return A Command that requires the shooter, hood, turret, and flywheel subsystems, and when executed, transitions the shooter to the IDLE state and holds this state until interrupted.
   */
  public Command idleCommand() {
    return Commands.run(() -> requestState(ShooterState.IDLE), this, hood, turret, flywheel);
  }

  public Command aimCommand() {
      return Commands.run(() -> requestState(ShooterState.AIM), this, hood, turret, flywheel);
  }

  public Command straightCommand() {
      return Commands.run(() -> requestState(ShooterState.STRAIGHT), this, hood, turret, flywheel);
  }

  public Command trenchCommand() {
      return Commands.run(() -> requestState(ShooterState.TRENCH), this, hood);
  }

  /**
   * Manual control command for the turret.
   * @param input A Supplier that provides the desired turret angle as a Rotation2d, typically based on joystick input or other manual controls.
   * @return A Command that requires the shooter and turret subsystems, and when executed, updates the turret's target angle based on the provided input. 
   */
  public Command manualTurretCommand(Supplier<Rotation2d> input) {
    return Commands.runEnd(() -> {
        requestState(ShooterState.MANUAL);
        turret.setSetpoint(input.get());
      },
      this::stop,
      turret);
  }

  /**
   * Manual control command for the hood.
   * @param setpoint A Supplier that provides the desired hood angle as a Rotation2d, typically based on joystick input or other manual controls.
   * @return A Command that requires the shooter and hood subsystems, and when executed, updates the hood's target angle based on the provided input.
   */
  public Command manualHoodCommand(Supplier<Rotation2d> setpoint) {
      return Commands.runEnd(() -> {
          requestState(ShooterState.MANUAL);
          hood.setSetpoint(setpoint.get());
        },
        this::stop,
        hood);
  }

  /**
   * Manual control command for the flywheel.
   * @param setpoint A DoubleSupplier that provides the manual input for flywheel control, representing the desired flywheel speed in rotations per second.
   * @return A Command that requires the shooter and flywheel subsystems, and when executed, updates the flywheel's target speed based on the provided input.
   */
  public Command manualFlywheelCommand(Supplier<AngularVelocity> setpoint) {
      return Commands.runEnd(() -> {
          requestState(ShooterState.MANUAL);
          flywheel.setSetpoint(setpoint.get());
        },
       this::stop,
       flywheel);
  }

  public boolean isFlipping() {
    return turret.isFlipping();
  }


}