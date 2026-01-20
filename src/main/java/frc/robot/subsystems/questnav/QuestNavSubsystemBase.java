package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Abstract base for QuestNav subsystem implementations. Allows using a no-op substitute in
 * SIM/replay while keeping the real implementation for REAL.
 */
public abstract class QuestNavSubsystemBase extends SubsystemBase {
  /** Returns the latest robot pose according to QuestNav (or fallback). */
  public abstract Pose2d getRobotPose();

  /** Returns whether QuestNav is actively tracking. */
  public abstract boolean isTracking();

  /** Allows external reset of the QuestNav pose. */
  public abstract void resetPose(Pose2d newRobotPose);
}
