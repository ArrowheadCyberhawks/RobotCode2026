package frc.robot.subsystems.shooter;

// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;
import frc.robot.Constants;
import frc.robot.util.FieldConstants;
import frc.robot.util.geometry.AllianceFlipUtil;
import frc.robot.util.geometry.GeomUtil;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {
    private static ShotCalculator instance;

    // Moving-average filters to smooth out noisy angle computations. Using the
    // same window for turret and hood keeps outputs stable for control and
    // derivative calculations.
    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
    private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

    // Last values used for simple derivative computations (angular velocities).
    private Rotation2d lastTurretAngle;
    private double lastHoodAngle;

    // Current computed setpoints (populated by getData()).
    private Rotation2d turretAngle;
    private double hoodAngle = Double.NaN;
    private double turretVelocity;
    private double hoodVelocity;

    // Configurable base target for shots (defaults to the hub). The baseTarget
    // is stored in field coordinates; getData() applies alliance flipping so
    // callers always pass a single un-flipped target.
    private Translation2d target = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    // Suppliers for injected state (defaults are zero/no-op). RobotContainer should
    // inject real suppliers.
    private Supplier<ChassisSpeeds> fieldVelocitySupplier = () -> new ChassisSpeeds(0.0, 0.0, 0.0);
    private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
    private Supplier<ChassisSpeeds> robotRelativeVelocitySupplier = () -> new ChassisSpeeds(0.0, 0.0, 0.0);

    /**
     * Singleton accessor. Lightweight and thread-unsafe (intended for robot
     * code where single-threaded access is the norm). Callers should retrieve
     * the instance and then call getData() to compute/read setpoints.
     */
    public static ShotCalculator getInstance() {
        if (instance == null)
            instance = new ShotCalculator();
        return instance;
    }

    public record ShotData(
            boolean isValid,
            Rotation2d turretAngle,
            double turretVelocity,
            double hoodAngle,
            double hoodVelocity,
            double flywheelSpeed) {
    }

    // Cached output from the last getData() call. Clearing this allows callers
    // to force recomputation when inputs change.
    private ShotData latestData = null;

    // Configuration and lookup tables used by the calculator. These maps are
    // populated with empirically-determined values that map distance ->
    // desired hood angle and distance -> flywheel speed. The tofMap maps a
    // distance to an estimated time-of-flight for lookahead calculations.
    private static double minDistance;
    private static double maxDistance;
    private static double phaseDelay;
    private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap = 
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

    static {
        // Reasonable operating bounds for the shooter (meters) and a small
        // phase delay used to offset calculations for shooter processing time.
        minDistance = 1.34;
        maxDistance = 5.60;
        phaseDelay = 0.09; // started at .03, increased to 0.09 for better accuracy, will change based on

        // Populate the hood angle calibration map (distance -> angle). These
        // values should be tuned on the field; interpolation fills in values
        // between the points defined here.
        hoodAngleMap.put(1.34, Rotation2d.fromDegrees(23.1));
        hoodAngleMap.put(5.60, Rotation2d.fromDegrees(50.0));

        // Populate the flywheel speed calibration map (distance -> RPM).
        flywheelSpeedMap.put(1.34, 150.0);
        flywheelSpeedMap.put(5.60, 250.0);

        // Populate a small time-of-flight lookup table (distance -> seconds)
        // used in the lookahead loop to compensate for turret/robot motion.
        tofMap.put(5.60, 3.00);
        tofMap.put(1.34, 0.90);
    }

    public ShotData getData() {
        // Calculate estimated pose while accounting for time between calculation and
        // the shot
        Pose2d estimatedPose = poseSupplier.get();
        ChassisSpeeds robotRelativeVelocity = robotRelativeVelocitySupplier.get();
        estimatedPose = estimatedPose.exp(
                new Twist2d(
                        robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                        robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                        robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

        // Calculate distance from turret to target
        // Apply currently-configured target (default is the hub) with alliance flip
        Translation2d target = AllianceFlipUtil.apply(this.target);
        // Use the configured robot->turret transform from ShooterConstants (drop Z)
        var robotToTurretTrans = ShooterConstants.kRobotToTurretTransform.getTranslation();
        Pose2d turretPosition = estimatedPose.transformBy(
                new Transform2d(
                        new Translation2d(robotToTurretTrans.getX(), robotToTurretTrans.getY()),
                        new Rotation2d()));
        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

        // Calculate field relative turret velocity
        ChassisSpeeds robotVelocity = fieldVelocitySupplier.get();
        double robotAngle = estimatedPose.getRotation().getRadians();
        double turretVelocityX = robotVelocity.vxMetersPerSecond
                // + robotVelocity.omegaRadiansPerSecond
                * (robotToTurretTrans.getY() * Math.cos(robotAngle)
                        - robotToTurretTrans.getX() * Math.sin(robotAngle));
        double turretVelocityY = robotVelocity.vyMetersPerSecond
                // + robotVelocity.omegaRadiansPerSecond
                * (robotToTurretTrans.getX() * Math.cos(robotAngle)
                        - robotToTurretTrans.getY() * Math.sin(robotAngle));

        // Account for imparted velocity by robot (turret) to offset
        double timeOfFlight;
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = tofMap.get(lookaheadTurretToTargetDistance);
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            lookaheadPose = new Pose2d(
                    turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    turretPosition.getRotation());
            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        // Calculate parameters accounted for imparted velocity
        // Get field-relative angle from turret to target
        double fieldRelativeAngleRad = target.minus(lookaheadPose.getTranslation()).getAngle().getRadians();
        // Convert to robot-relative by subtracting robot heading
        double robotRelativeAngleRad = fieldRelativeAngleRad - estimatedPose.getRotation().getRadians();
        // Normalize to [-π, π]
        double rawTurretAngleRad = Math.atan2(Math.sin(robotRelativeAngleRad), Math.cos(robotRelativeAngleRad));

        // Filter the turret angle to smooth noisy measurements
        double filteredTurretAngleRad = turretAngleFilter.calculate(rawTurretAngleRad);

        turretAngle = Rotation2d.fromRadians(filteredTurretAngleRad);

        // Log calculated values for debugging
        Logger.recordOutput("ShotCalculator/RobotPose", estimatedPose);
        Logger.recordOutput("ShotCalculator/TargetPos", target);
        Logger.recordOutput("ShotCalculator/TurretPos", turretPosition.getTranslation());
        Logger.recordOutput("ShotCalculator/FieldRelativeAngle", fieldRelativeAngleRad);
        Logger.recordOutput("ShotCalculator/RobotRelativeAngle", robotRelativeAngleRad);
        Logger.recordOutput("ShotCalculator/RawTurretAngle", rawTurretAngleRad);
        Logger.recordOutput("ShotCalculator/FilteredTurretAngle", filteredTurretAngleRad);

        hoodAngle = hoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
        // Smooth hood angle as well
        hoodAngle = hoodAngleFilter.calculate(hoodAngle);

        if (lastTurretAngle == null)
            lastTurretAngle = turretAngle;
        if (Double.isNaN(lastHoodAngle))
            lastHoodAngle = hoodAngle;

        // Compute angular velocities (simple derivative on filtered angle)
        turretVelocity = (turretAngle.getRadians() - lastTurretAngle.getRadians()) / Constants.loopPeriodSecs;
        hoodVelocity = (hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs;

        lastTurretAngle = turretAngle;
        lastHoodAngle = hoodAngle;
        // Valid only when distance in range AND turret angle is within +/- 3/4*pi
        latestData = new ShotData(
                lookaheadTurretToTargetDistance >= minDistance && lookaheadTurretToTargetDistance <= maxDistance
                        && Math.abs(filteredTurretAngleRad) <= (3.0 / 4.0) * Math.PI,
                turretAngle,
                turretVelocity,
                hoodAngle,
                hoodVelocity,
                flywheelSpeedMap.get(lookaheadTurretToTargetDistance));

        // Log calculated values
        Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

        return latestData;
    }

    public void clearShotData() {
        latestData = null;
    }

    public void setFieldVelocitySupplier(Supplier<ChassisSpeeds> supplier) {
        this.fieldVelocitySupplier = supplier == null ? () -> new ChassisSpeeds(0.0, 0.0, 0.0) : supplier;
    }

    /**
     * Change the base (un-flipped) target used for shot calculations. Pass a
     * Translation2d
     * in field coordinates; alliance flipping is applied automatically in
     * getData().
     */
    public void setTarget(Translation2d newBaseTarget) {
        this.target = newBaseTarget == null ? FieldConstants.Hub.topCenterPoint.toTranslation2d() : newBaseTarget;
        clearShotData();
    }

    /** Reset the target back to the default hub location. */
    public void resetTargetToHub() {
        this.target = FieldConstants.Hub.topCenterPoint.toTranslation2d();
        clearShotData();
    }

    public void setPoseSupplier(Supplier<Pose2d> supplier) {
        this.poseSupplier = supplier == null ? () -> new Pose2d() : supplier;
    }

    public void setRobotRelativeVelocitySupplier(Supplier<ChassisSpeeds> supplier) {
        this.robotRelativeVelocitySupplier = supplier == null ? () -> new ChassisSpeeds(0.0, 0.0, 0.0) : supplier;
    }

    /**
     * Get the current target position (with alliance flipping applied).
     * Returns the target as a Translation2d in field coordinates.
     */
    public Translation2d getTarget() {
        return AllianceFlipUtil.apply(this.target);
    }
}
