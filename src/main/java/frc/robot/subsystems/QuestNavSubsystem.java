package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/**
 * Subsystem integrating QuestNav headset pose data into robot pose tracking.
 */
public class QuestNavSubsystem extends SubsystemBase {

    private final QuestNav questNav;
    private final CommandSwerveDrivetrain drivetrain;

    /** Transform from robot center → Quest mount (tune to your actual mount) */
    private static final Transform3d ROBOT_TO_QUEST = new Transform3d(
        0.0, // x offset
        0.0, // y offset
        0.0, // z offset
        new edu.wpi.first.math.geometry.Rotation3d(0.0, 0.0, 0.0) // rotation offset
    );

    /** Last known robot pose from QuestNav */
    private Pose3d lastRobotPose = new Pose3d();

    /** Vision standard deviations */
    private static final Matrix<N3, N1> QUESTNAV_STD_DEVS =
        VecBuilder.fill(
            0.02, // X noise (m)
            0.02, // Y noise (m)
            0.035 // rotation (rad)
        );

    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.questNav = new QuestNav();
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // Required for receiving QuestNav data
        questNav.commandPeriodic();

        if (questNav.isTracking()) {
            PoseFrame[] frames = questNav.getAllUnreadPoseFrames();

            for (PoseFrame frame : frames) {

                Pose3d questPose = frame.questPose3d();
                double timestamp = frame.dataTimestamp();

                // Convert Quest headset pose to robot pose
                Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());

                // Track last pose for getters
                lastRobotPose = robotPose;

                // Feed into drivetrain estimator
                drivetrain.addVisionMeasurement(
                    robotPose.toPose2d(),
                    Utils.fpgaToCurrentTime(timestamp),
                    QUESTNAV_STD_DEVS
                );
            }
        }
    }

    /** Returns the latest robot pose according to QuestNav. */
    public Pose2d getRobotPose() {
        return lastRobotPose.toPose2d();
    }

    /** Returns whether QuestNav is actively tracking. */
    public boolean isTracking() {
        return questNav.isTracking();
    }

    /** Allows external reset of the QuestNav pose. */
    public void resetPose(Pose3d newRobotPose) {
        Pose3d newQuestPose = newRobotPose.transformBy(ROBOT_TO_QUEST);
        questNav.setPose(newQuestPose);
    }
}
