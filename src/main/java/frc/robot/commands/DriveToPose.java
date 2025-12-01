// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// originally from https://github.com/Mechanical-Advantage/RobotCode2023

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

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

/**
 * This command, when executed, instructs the drivetrain subsystem to drive to
 * the specified pose in
 * a straight line. The execute method invokes the drivetrain subsystem's drive
 * method. For
 * following a predetermined path, refer to the FollowPath Command class. For
 * generating a path on
 * the fly and following that path, refer to the MoveToPose Command class.
 *
 * <p>
 * Requires: the Drivetrain subsystem
 *
 * <p>
 * Finished When: the robot is at the specified pose (within the specified
 * tolerances)
 *
 * <p>
 * At End: stops the drivetrain
 */
public class DriveToPose extends Command {
    public final CommandSwerveDrivetrain drivetrain;
    public final SwerveRequest.FieldCentricFacingAngle driveRequest;
    private final Supplier<Pose2d> poseSupplier;
    private Pose2d targetPose;
    private int atTargetCount = 0;

    private boolean running = false;
    private Timer timer;

    private static final double driveKp = 7.5;
    private static final double driveKd = 0.0;
    private static final double driveKi = 0.0;
    private static final double driveMaxVelocity = 2; //replace with constant value when safe to use lol
    private static final double driveMaxAcceleration = 1;
    private static final double driveTolerance = 0.05;
    private static final double thetaTolerance = 5.0;
    private static final double timeout = 5.0;

    private static final TrapezoidProfile.Constraints driveConstraints = new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration);

    private final ProfiledPIDController xController = new ProfiledPIDController(driveKp, driveKi, driveKd, driveConstraints, DriveConstants.kLoopPeriodSeconds);
    private final ProfiledPIDController yController = new ProfiledPIDController(driveKp, driveKi, driveKd, driveConstraints, DriveConstants.kLoopPeriodSeconds);

    /**
     * Constructs a new DriveToPose command that drives the robot in a straight line
     * to the specified pose.
     * A pose supplier is specified instead of a pose since the target pose may not
     * be known when this command is created.
     *
     * @param drivetrain   the drivetrain subsystem required by this command
     * @param poseSupplier a supplier that returns the pose to drive to
     */
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

    /**
     * This method is invoked once when this command is scheduled.
     * It resets all the PID controllers and initializes the current and target poses.
     * It is critical that this initialization occurs in this method and not the constructor as this object is constructed well before the command is scheduled
     * and the robot's pose will definitely have changed and the target pose may not be known until this command is scheduled.
     */
    @Override
    public void initialize() {
        // Reset all controllers
        Pose2d currentPose = drivetrain.getPose();
        xController.reset(currentPose.getX());
        yController.reset(currentPose.getY());
        xController.setTolerance(driveTolerance);
        yController.setTolerance(driveTolerance);
        this.targetPose = poseSupplier.get();
        xController.setGoal(this.targetPose.getX());
        yController.setGoal(this.targetPose.getY());

        // "horrible inefficient garbage telemetry code" - Grant
        SmartDashboard.putNumberArray("Robot Pose", new double[] { targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees() });

        this.timer.restart();
    }

    /**
     * This method is invoked periodically while this command is scheduled.
     * It calculates the velocities based on the current and target poses and uses the drivetrain subsystem's drive method.
     */
    @Override
    public void execute() {
        // set running to true in this method to capture that the calculate method has been invoked on the PID controllers.
        // This is important since these controllers will return true for atGoal if the calculate method has not yet been invoked.
        running = true;

        Pose2d currentPose = drivetrain.getPose();

        double xVelocity = xController.calculate(currentPose.getX());
        double yVelocity = yController.calculate(currentPose.getY());
        if (xController.atGoal()) xVelocity = 0;
        if (yController.atGoal()) yVelocity = 0;

        drivetrain.setControl(
            driveRequest
                .withVelocityX(xVelocity) // Drive forward with negative Y (forward)
                .withVelocityY(yVelocity) // Drive left with negative X (left)
                .withTargetDirection(targetPose.getRotation()));
    }

    /**
     * This method returns true if the command has finished. It is invoked periodically while this command is scheduled (after execute is invoked).
     * @return true if the command has finished
     */
    @Override
    public boolean isFinished() {
        //telemetry
        SmartDashboard.putBoolean("DriveToPose/xErr", xController.atGoal());
        SmartDashboard.putBoolean("DriveToPose/yErr", yController.atGoal());
        SmartDashboard.putBoolean("DriveToPose/tErr", isThetaAtGoal());

        boolean isAtTarget = (running && xController.atGoal() && yController.atGoal() && isThetaAtGoal());

        if (isAtTarget) {
            atTargetCount++;
        } else {
            atTargetCount = 0;
        }

        //makes sure the PID controllers are calculating and that each of the controllers is at their goal.
        //Important since these controllers will return true for atGoal if the calculate method has not yet been invoked.
        return this.timer.hasElapsed(timeout) || atTargetCount >= 15;
    }

    private boolean isThetaAtGoal() {
        return Math.abs(targetPose.getRotation().minus(drivetrain.getPose().getRotation()).getDegrees()) < thetaTolerance;
    }

    /**
     * This method will be invoked when this command finishes or is interrupted.
     * It stops the motion of the drivetrain.
     *
     * @param interrupted true if the command was interrupted by another command
     *                    being scheduled
     */
    @Override
    public void end(boolean interrupted) {
        //break
        drivetrain.setControl(
                driveRequest
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withTargetDirection(drivetrain.getPose().getRotation()));
        running = false;
    }
}
