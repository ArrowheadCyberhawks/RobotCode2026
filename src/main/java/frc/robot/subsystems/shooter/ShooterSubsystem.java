package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.rev.HoodSubsystemNeo;
import frc.robot.subsystems.shooter.rev.TurretSubsystemNeo;
import frc.robot.subsystems.shooter.talonfx.FlywheelSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  
  public enum ShooterState {
    IDLE,
    STRAIGHT,
    AIM,
    TRENCH
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
    desiredState = ShooterState.IDLE;
    currentState = ShooterState.IDLE;
  }

  public boolean areAllSubsystemsAtGoal() {
    return flywheel.isAtGoal() && hood.isAtGoal() && turret.isAtGoal();
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
    }
  }
}