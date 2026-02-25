package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.rev.HoodSubsystemNeo;
import frc.robot.subsystems.shooter.rev.TurretSubsystemNeo;
import frc.robot.subsystems.shooter.talonfx.FlywheelSubsystem;
import frc.robot.subsystems.shooter.talonfx.HoodSubsystem;
import frc.robot.subsystems.shooter.talonfx.TurretSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

/**
 * State machine that coordinates the three TalonFX shooter subsystems:
 * Flywheel, Hood, and Turret.
 * 
 * State Transitions:
 * - IDLE -> AIM (start aiming and spinning up)
 * - IDLE -> STRAIGHT (align turret straight, spin up for direct shot)
 * - AIM -> SHOOT (when all subsystems at goal)
 * - STRAIGHT -> SHOOT (when all subsystems at goal)
 * - SHOOT -> IDLE (after shooting)
 * - SHOOT -> STRAIGHT (quick realignment)
 */
public class ShooterSubsystem extends SubsystemBase {
  
  public enum ShooterState {
    IDLE,
    STRAIGHT,
    AIM,
    SHOOT
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
    // Update state transitions
    updateState();

    // Execute current state behavior
    executeState();

    // Log state information
    Logger.recordOutput("Shooter/State", currentState.name());
    Logger.recordOutput("Shooter/DesiredState", desiredState.name());
    Logger.recordOutput("Shooter/AllAtGoal", areAllSubsystemsAtGoal());
  }

  /**
   * Request a state transition. The state machine will validate and execute
   * the transition according to the rules.
   */
  public void requestState(ShooterState state) {
    // Validate state transition
    if (isValidTransition(currentState, state)) {
      desiredState = state;
    } else {
      Logger.recordOutput("Shooter/InvalidTransition", 
          String.format("%s -> %s", currentState.name(), state.name()));
    }
  }

  /**
   * Get the current shooter state.
   */
  public ShooterState getState() {
    return currentState;
  }

  /**
   * Check if all subsystems are at their goals.
   */
  public boolean areAllSubsystemsAtGoal() {
    return flywheel.isAtGoal() && hood.isAtGoal() && turret.isAtGoal();
  }

  /**
   * Convenience method to start aiming sequence.
   */
  public void startAiming() {
    requestState(ShooterState.AIM);
  }

  /**
   * Convenience method to aim straight ahead.
   */
  public void aimStraight() {
    requestState(ShooterState.STRAIGHT);
  }

  /**
   * Convenience method to return to idle.
   * This always works regardless of current state — safety override.
   */
  public void stop() {
    desiredState = ShooterState.IDLE;
    currentState = ShooterState.IDLE;
  }

  /**
   * Check if the shooter is ready to fire (in SHOOT state).
   */
  public boolean isReadyToShoot() {
    return currentState == ShooterState.SHOOT;
  }

  private void updateState() {
    switch (currentState) {
      case IDLE:
        // IDLE can transition to AIM or STRAIGHT
        if (desiredState == ShooterState.AIM || desiredState == ShooterState.STRAIGHT) {
          currentState = desiredState;
        }
        break;

      case STRAIGHT:
      case AIM:
        // AIM/STRAIGHT automatically transition to SHOOT when all subsystems ready
        if (areAllSubsystemsAtGoal()) {
          currentState = ShooterState.SHOOT;
          desiredState = ShooterState.SHOOT;
        }
        break;

      case SHOOT:
        // SHOOT can transition to IDLE or STRAIGHT
        if (desiredState == ShooterState.IDLE) {
          currentState = ShooterState.IDLE;
        } else if (desiredState == ShooterState.STRAIGHT) {
          currentState = ShooterState.STRAIGHT;
        }
        break;
    }
  }

  private void executeState() {
    switch (currentState) {
      case IDLE:
        // Stop all subsystems directly — no commands
        flywheel.stop();
        hood.stopHood();
        turret.stopTurret();
        break;

      case STRAIGHT:
        // Turret aims straight (0 degrees)
        turret.moveTurretToDegrees(0.0);
        
        // Get shot data for straight shot
        var straightData = ShotCalculator.getInstance().getData();
        if (straightData != null && straightData.isValid()) {
          // Use calculated flywheel speed and hood angle
          flywheel.runVelocity(RadiansPerSecond.of(straightData.flywheelSpeed()));
          hood.moveHoodToDegrees(Math.toDegrees(straightData.hoodAngle()));
        }
        break;

      case AIM:
        // All subsystems track the target from ShotCalculator
        var aimData = ShotCalculator.getInstance().getData();
        if (aimData != null && aimData.isValid()) {
          flywheel.runVelocity(RadiansPerSecond.of(aimData.flywheelSpeed()));
          hood.moveHoodToDegrees(Math.toDegrees(aimData.hoodAngle()));
          //turret.moveTurretToRadians(aimData.turretAngle().getRadians());
        }
        break;

      case SHOOT:
        // Maintain current positions by continuing to track
        var shootData = ShotCalculator.getInstance().getData();
        if (shootData != null && shootData.isValid()) {
          flywheel.runVelocity(RadiansPerSecond.of(shootData.flywheelSpeed()));
          hood.moveHoodToDegrees(Math.toDegrees(shootData.hoodAngle()));
          //turret.moveTurretToRadians(shootData.turretAngle().getRadians());
        }
        break;
    }
  }

  private boolean isValidTransition(ShooterState from, ShooterState to) {
    switch (from) {
      case IDLE:
        // IDLE can only go to AIM or STRAIGHT
        return to == ShooterState.AIM || to == ShooterState.STRAIGHT;
      
      case STRAIGHT:
      case AIM:
        // AIM/STRAIGHT automatically goes to SHOOT (handled in updateState)
        // Can also be requested to return to IDLE
        return to == ShooterState.SHOOT || to == ShooterState.IDLE;
      
      case SHOOT:
        // SHOOT can go to IDLE or STRAIGHT
        return to == ShooterState.IDLE || to == ShooterState.STRAIGHT;
      
      default:
        return false;
    }
  }
}
