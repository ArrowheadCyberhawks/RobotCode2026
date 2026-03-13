package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.rev.HoodSubsystemNeo;
import frc.robot.subsystems.shooter.rev.TurretSubsystemNeo;
import frc.robot.subsystems.shooter.talonfx.FlywheelSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  
  public enum ShooterState {
    IDLE,
    STRAIGHT,
    AIM,
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
    // desiredState = ShooterState.IDLE;
    // currentState = ShooterState.IDLE;
    requestState(ShooterState.IDLE);
  }

  public boolean areAllSubsystemsAtGoal() {
    return flywheel.isAtGoal() && hood.isAtGoal() && turret.isAtGoal();
  }

  public HoodSubsystemNeo getHood() {
    return hood;
  }

  public TurretSubsystemNeo getTurret() {
      return turret;
  }

  public FlywheelSubsystem getFlywheel() {
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
        hood.stopHood();
        turret.stopTurret();
        break;

      case STRAIGHT:
        turret.setTurretTarget(new Rotation2d());

        var straightData = ShotCalculator.getInstance().getData();
        if (straightData != null && straightData.isValid()) {
          flywheel.runVelocity(RadiansPerSecond.of(straightData.flywheelSpeed()));
          hood.moveHoodTo(straightData.hoodAngle());
        }
        break;

      case AIM:
        var aimData = ShotCalculator.getInstance().getData();
        if (aimData != null && aimData.isValid()) {
          flywheel.runVelocity(RadiansPerSecond.of(aimData.flywheelSpeed()));
          hood.moveHoodTo(aimData.hoodAngle());
          turret.setTurretTarget(aimData.turretAngle());
        }
        break;

      case TRENCH:
        hood.moveHoodTo(ShooterConstants.HoodPosition.STOW);
        break;

      case MANUAL:
        break;

    }
  }

  // Simplification so you don't have to use run()

  public Command idleCommand() {
    return runOnce(() -> requestState(ShooterState.IDLE));
  }

  public Command aimCommand() {
      return run(() -> requestState(ShooterState.AIM));
  }

  public Command straightCommand() {
      return run(() -> requestState(ShooterState.STRAIGHT));
  }

  public Command trenchCommand() {
      return run(() -> requestState(ShooterState.TRENCH));
  }

  // Manual Controls within shootersubsystem to avoid touching the specific subsystems
  public Command manualTurret(DoubleSupplier input) {
    return run(() -> {
        requestState(ShooterState.MANUAL);
        turret.setTurretTarget(
            turret.getTurretTarget().plus(
                Rotation2d.fromDegrees(input.getAsDouble() * 2.0)
            )
        );
    });
  }

  public Command manualHood(DoubleSupplier input) {
      return run(() -> {
          requestState(ShooterState.MANUAL);
          hood.moveHoodTo(
              hood.getHoodTargetAngle().plus(
                  Rotation2d.fromDegrees(input.getAsDouble())
              )
          );
      });
  }

  public Command manualFlywheel(DoubleSupplier speed) {
      return run(() -> {
          requestState(ShooterState.MANUAL);
          flywheel.runVelocity(RadiansPerSecond.of(speed.getAsDouble()));
      });
  }


}