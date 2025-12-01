
# DriveToPose Docs

## Package and Imports

```java
package frc.robot.commands;
```

* The command resides in the `commands` package of the robot project.
* It depends on subsystems, WPILib math libraries, and CTRE swerve control classes.

```java
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;
```

* **SwerveRequest**: Used to generate requests for swerve module velocity, rotation, and direction.
* **ProfiledPIDController**: PID controller with trapezoidal velocity constraints for smooth motion.
* **Pose2d**: Represents a 2D position and rotation.
* **Timer**: Used to implement command timeout.
* **SmartDashboard**: Sends telemetry to dashboard for debugging.
* **Supplier<Pose2d>**: Allows the target pose to be dynamically supplied at runtime.

---

## Class Declaration

```java
public class DriveToPose extends Command {
```

* Extends `Command` from WPILib to define a reusable robot action.
* Requires the **drivetrain subsystem** to control swerve modules.

---

## Member Variables

```java
public final CommandSwerveDrivetrain drivetrain;
public final SwerveRequest.FieldCentricFacingAngle driveRequest;
private final Supplier<Pose2d> poseSupplier;
private Pose2d targetPose;
private int atTargetCount = 0;
private boolean running = false;
private Timer timer;
```

### Explanation:

* `drivetrain`: Reference to the swerve drivetrain subsystem.
* `driveRequest`: Template request for driving the robot in **field-centric mode**.
* `poseSupplier`: Supplies the target pose dynamically.
* `targetPose`: The current target pose used for calculations.
* `atTargetCount`: Counts consecutive cycles the robot is within tolerance.
* `running`: Tracks if the command is currently executing.
* `timer`: Enforces a timeout for safety.

---

## PID Controller Constants

```java
private static final double driveKp = 7.5;
private static final double driveKd = 0.0;
private static final double driveKi = 0.0;
private static final double driveMaxVelocity = 3.5;
private static final double driveMaxAcceleration = 3.0;
private static final double driveTolerance = 0.04;
private static final double thetaTolerance = 5.0;
private static final double timeout = 5.0;
```

* **PID Gains**:

  * `Kp`: Proportional gain for position correction.
  * `Ki` / `Kd`: Integral and derivative gains (unused here but available for tuning).
* **Motion Constraints**:

  * `driveMaxVelocity` / `driveMaxAcceleration`: Used in trapezoidal motion profiling.
* **Tolerances**:

  * `driveTolerance`: Position tolerance in meters.
  * `thetaTolerance`: Angular tolerance in degrees.
* **Timeout**:

  * Safety mechanism to end command after a fixed duration.

---

## Profiled PID Controllers

```java
private final ProfiledPIDController xController = new ProfiledPIDController(
        driveKp, driveKi, driveKd,
        new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration),
        DriveConstants.kLoopPeriodSeconds);

private final ProfiledPIDController yController = new ProfiledPIDController(
        driveKp, driveKi, driveKd,
        new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration),
        DriveConstants.kLoopPeriodSeconds);
```

* Two independent controllers control **X and Y positions**.
* Trapezoidal constraints enforce **smooth acceleration** and **maximum velocity**.
* `kLoopPeriodSeconds` is the periodic loop duration of the drivetrain.

---

## Constructor

```java
public DriveToPose(
        CommandSwerveDrivetrain drivetrain,
        Supplier<Pose2d> poseSupplier,
        SwerveRequest.FieldCentricFacingAngle driveRequest) {
    this.driveRequest = driveRequest;
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    this.timer = new Timer();
    addRequirements(drivetrain);
}
```

* Initializes the command with:

  * **Drivetrain subsystem**
  * **Dynamic pose supplier**
  * **Drive request template**
* Adds a **requirement** on the drivetrain so that WPILib knows only one command can control it at a time.

---

## initialize()

```java
@Override
public void initialize() {
    Pose2d currentPose = drivetrain.getPose();
    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    xController.setTolerance(driveTolerance);
    yController.setTolerance(driveTolerance);
    this.targetPose = poseSupplier.get();
    xController.setGoal(this.targetPose.getX());
    yController.setGoal(this.targetPose.getY());

    SmartDashboard.putNumberArray("Robot Pose",
            new double[] { targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees() });

    this.timer.restart();
}
```

* **Purpose**: Prepare PID controllers and initialize target pose when the command starts.
* **Key points**:

  * Reset PID state to current robot position.
  * Set tolerances to define "at goal".
  * Fetch target pose from the supplier.
  * Start a timer for command timeout.
  * Send telemetry to SmartDashboard.

---

## execute()

```java
@Override
public void execute() {
    running = true;

    Pose2d currentPose = drivetrain.getPose();

    double xVelocity = xController.calculate(currentPose.getX());
    double yVelocity = yController.calculate(currentPose.getY());
    if (xController.atGoal()) xVelocity = 0.0;
    if (yController.atGoal()) yVelocity = 0.0;

    drivetrain.setControl(
            driveRequest
                    .withVelocityX(xVelocity)
                    .withVelocityY(yVelocity)
                    .withTargetDirection(targetPose.getRotation()));
}
```

* Called periodically during command execution.
* **Calculates velocities** in X and Y using the PID controllers.
* Stops motion in axes that are already at goal.
* Sets drivetrain control using the **SwerveRequest** object:

  * `withVelocityX` / `withVelocityY` control translation.
  * `withTargetDirection` ensures robot faces the desired heading.

---

## isFinished()

```java
@Override
public boolean isFinished() {
    SmartDashboard.putBoolean("DriveToPose/xErr", xController.atGoal());
    SmartDashboard.putBoolean("DriveToPose/yErr", yController.atGoal());
    SmartDashboard.putBoolean("DriveToPose/tErr", isThetaAtGoal());

    boolean isAtTarget = (running && xController.atGoal() && yController.atGoal() && isThetaAtGoal());
    if (isAtTarget) {
        atTargetCount++;
    } else {
        atTargetCount = 0;
    }

    return this.timer.hasElapsed(timeout) || atTargetCount >= 15;
}
```

* Returns `true` if the robot has reached the target pose or the timeout has elapsed.
* Uses `atTargetCount` to ensure stability (robot must be within tolerances for 15 cycles).

---

### Helper: isThetaAtGoal()

```java
private boolean isThetaAtGoal() {
    return Math.abs(targetPose.getRotation().minus(drivetrain.getPose().getRotation()).getDegrees()) < thetaTolerance;
}
```

* Checks if robot's heading is within `thetaTolerance` of the target rotation.

---

## end()

```java
@Override
public void end(boolean interrupted) {
    drivetrain.setControl(
            driveRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withTargetDirection(drivetrain.getPose().getRotation()));
    running = false;
}
```

* Stops the drivetrain when command finishes or is interrupted.
* Maintains robot orientation at last heading.

---

## Summary

`DriveToPose` allows your robot to:

1. Drive to a **specific field position and orientation**.
2. Use **PID controllers** for smooth, precise motion.
3. Automatically **stop at the target** and enforce **stability**.
4. Report telemetry to SmartDashboard for debugging.

It can be combined with vision or other commands by updating the `poseSupplier` dynamically.

---

## Potential Extensions

* **Vision integration**: Replace `poseSupplier` with a Limelight-based supplier to drive toward a target tag.
* **Dynamic obstacle avoidance**: Incorporate a dynamic path planner to update `targetPose` in real time.
* **Enhanced PID tuning**: Add `Kd` and `Ki` values to smooth overshoot or correct steady-state errors.

---

## References

* [WPILib Command-based programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/commands/commands.html)
* [CTRE Phoenix Swerve API](https://www.ctr-electronics.com/)
* [ProfiledPIDController Documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/profiled-pid-controller.html)
* [FRC Pose2d / Rotation2d](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/pose2d.html)

