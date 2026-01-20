package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.Drive;

/**
 * No-op substitute for QuestNavSubsystem used in SIM and replay/default modes. Provides the same
 * public API but does not attempt to use QuestNav native bindings.
 */
public class QuestNavSubsystemSim extends QuestNavSubsystemBase {
  private final Drive drive;

  public QuestNavSubsystemSim(Drive drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    // intentionally empty
  }

  /** Returns a default pose (current drivetrain pose) as a safe fallback. */
  public Pose2d getRobotPose() {
    return drive.getPose();
  }

  /** Always returns false in simulation/replay. */
  public boolean isTracking() {
    return false;
  }

  /** No-op reset. */
  public void resetPose(Pose2d newRobotPose) {
    // no-op in substitute
  }
}
