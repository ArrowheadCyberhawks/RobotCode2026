package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

/**
 * DriveToPose command: drives the provided Drive subsystem to a target Pose2d using
 * ProfiledPIDControllers for X, Y, and Theta. Uses field-relative velocities and sends chassis
 * speeds to Drive.runVelocity().
 */
public class DriveToPose extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> poseSupplier;

  // Tunable parameters (conservative defaults)
  private static final double DRIVE_KP = 3.0;
  private static final double DRIVE_KI = 0.0;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_MAX_VEL = 2.0; // meters/sec
  private static final double DRIVE_MAX_ACCEL = 1.0; // meters/sec^2

  private static final double TURN_KP = 4.0;
  private static final double TURN_KI = 0.0;
  private static final double TURN_KD = 0.0;
  private static final double TURN_MAX_VEL = Math.PI; // rad/sec
  private static final double TURN_MAX_ACCEL = Math.PI; // rad/sec^2

  private static final double DRIVE_TOLERANCE = 0.05; // meters
  private static final double THETA_TOLERANCE = 0.03; // radians (~1.7 deg)
  private static final double TIMEOUT_SECONDS = 60.0;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private final Timer timer = new Timer();
  private Pose2d targetPose;
  private boolean running = false;
  private int atTargetCount = 0;

  public DriveToPose(Drive drive, Supplier<Pose2d> poseSupplier) {
    this.drive = drive;
    this.poseSupplier = poseSupplier;
    // The project uses the older Command API; addRequirements is not available on that
    // base here. The scheduler will still run this command, but it won't automatically
    // claim the subsystem. If you later upgrade WPILib, convert to CommandBase and
    // uncomment addRequirements(drive).
    // addRequirements(drive);

    var driveConstraints = new TrapezoidProfile.Constraints(DRIVE_MAX_VEL, DRIVE_MAX_ACCEL);
    var turnConstraints = new TrapezoidProfile.Constraints(TURN_MAX_VEL, TURN_MAX_ACCEL);

    xController = new ProfiledPIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD, driveConstraints);
    yController = new ProfiledPIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD, driveConstraints);
    thetaController = new ProfiledPIDController(TURN_KP, TURN_KI, TURN_KD, turnConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    Pose2d current = drive.getPose();
    xController.reset(current.getX());
    yController.reset(current.getY());
    thetaController.reset(current.getRotation().getRadians());

    xController.setTolerance(DRIVE_TOLERANCE);
    yController.setTolerance(DRIVE_TOLERANCE);
    thetaController.setTolerance(THETA_TOLERANCE);

    targetPose = poseSupplier.get();
    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
    thetaController.setGoal(targetPose.getRotation().getRadians());

    timer.restart();
    running = false;
    atTargetCount = 0;
  }

  @Override
  public void execute() {
    running = true;
    Pose2d current = drive.getPose();

    double xVel = xController.calculate(current.getX());
    double yVel = yController.calculate(current.getY());
    double thetaVel = thetaController.calculate(current.getRotation().getRadians());

    if (xController.atGoal()) xVel = 0.0;
    if (yController.atGoal()) yVel = 0.0;
    if (thetaController.atGoal()) thetaVel = 0.0;

    // xVel and yVel are field-frame velocities (m/s). Convert to chassis speeds
    ChassisSpeeds desired = new ChassisSpeeds(xVel, yVel, thetaVel);
    // Convert field-relative speeds to robot-relative using current rotation and send
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(desired, drive.getRotation()));
  }

  @Override
  public boolean isFinished() {
    boolean isAtTarget =
        running && xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    if (isAtTarget) {
      atTargetCount++;
    } else {
      atTargetCount = 0;
    }
    return timer.hasElapsed(TIMEOUT_SECONDS) || atTargetCount >= 10;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    drive.runVelocity(new ChassisSpeeds());
    running = false;
  }
}
