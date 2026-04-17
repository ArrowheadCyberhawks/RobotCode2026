package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;

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

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.concurrent.TimeoutException;
import java.util.function.Supplier;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.field.FieldConstants;
import frc.robot.util.geometry.AllianceFlipUtil;
import frc.robot.util.geometry.GeomUtil;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {
    private static ShotCalculator instance;

    // Moving-average filters to smooth out noisy angle computations. Using the
    // same window for turret and hood keeps outputs stable for control and
    // derivative calculations.
    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.DriveConstants.kLoopPeriodSeconds));
    private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.DriveConstants.kLoopPeriodSeconds));

    private final LoggedTunableNumber velocityOffset = new LoggedTunableNumber("ShotCalculator/VelocityOffset", 0.0);
    private final LoggedTunableNumber hoodAngleOffset = new LoggedTunableNumber("ShotCalculator/hoodAngleOffset", 0.0);

    // Last values used for simple derivative computations (angular velocities).
    private Rotation2d lastTurretAngle;
    private Rotation2d lastHoodAngle;

    // Current computed setpoints (populated by getData()).
    private Rotation2d turretAngle;
    private Rotation2d hoodAngle;
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

    private ChassisSpeeds lastFieldVelocity = new ChassisSpeeds();
    private double ax = 0.0;
    private double ay = 0.0;

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
            Rotation2d hoodAngle,
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

        private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMapPassing = 
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap flywheelSpeedMapPassing = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap tofMapPassing = new InterpolatingDoubleTreeMap();

    static {
        // Reasonable operating bounds for the shooter (meters) and a small
        // phase delay used to offset calculations for shooter processing time.
        minDistance = 2.16;
        maxDistance = 5.60;
        phaseDelay = 0.06; // started at .03, increased to 0.09 for better accuracy, will change based on

        // Populate the hood angle calibration map (distance -> angle). These
        // values should be tuned on the field; interpolation fills in values
        // between the points defined here.
        // Meters
        hoodAngleMap.put(1.14, Rotation2d.fromDegrees(15)); //Change to 15 once hood can go down that low
        hoodAngleMap.put(1.34, Rotation2d.fromDegrees(15)); //Change to 15 once hood can go down that low
        hoodAngleMap.put(2.43, Rotation2d.fromDegrees(23.5));
        hoodAngleMap.put(3.04, Rotation2d.fromDegrees(25.0));
        hoodAngleMap.put(4.00, Rotation2d.fromDegrees(30.0));
        hoodAngleMap.put(5.5, Rotation2d.fromDegrees(35.5)); //adjusted to 30.0 for WILAX playoff 11
        hoodAngleMap.put(9.14, Rotation2d.fromDegrees(45));
        hoodAngleMap.put(12.95, Rotation2d.fromDegrees(45));

        // Populate the flywheel speed calibration map (distance -> RPS). (rotations per second)
        //Distance is in Meters
        flywheelSpeedMap.put(1.30,22.02);
        flywheelSpeedMap.put(2.43, 24.0); 
        flywheelSpeedMap.put(3.04, 25.5);
        flywheelSpeedMap.put(4.00, 27.0);
        flywheelSpeedMap.put(5.5, 34.0);
        flywheelSpeedMap.put(9.14, 38.0);
        flywheelSpeedMap.put(12.95, 55.0); 


        // Populate a small time-of-flight lookup table (distance -> seconds)
        // used in the lookahead loop to compensate for turret/robot motion.

        tofMap.put(1.35, 1.10);
        tofMap.put(2.43, 1.21);
        tofMap.put(3.04, 1.32);
        // tofMap.put(3.5, 1.36);
        tofMap.put(4.00, 1.40);
        tofMap.put(5.5, 1.50);
        tofMap.put(9.64, 1.70);
        tofMap.put(13.95, 2.20);

        // For now, use the same calibration points for passing shots. These
        // passing maps can be tuned independently later; copy values to
        // provide functional defaults.
        hoodAngleMapPassing.put(1.14, Rotation2d.fromDegrees(30));
        hoodAngleMapPassing.put(1.34, Rotation2d.fromDegrees(30));
        hoodAngleMapPassing.put(2.43, Rotation2d.fromDegrees(30));
        hoodAngleMapPassing.put(3.04, Rotation2d.fromDegrees(30.0));
        hoodAngleMapPassing.put(4.00, Rotation2d.fromDegrees(35));
        hoodAngleMapPassing.put(5.5, Rotation2d.fromDegrees(35.0));
        hoodAngleMapPassing.put(9.14, Rotation2d.fromDegrees(45));
        hoodAngleMapPassing.put(12.95, Rotation2d.fromDegrees(45));

        flywheelSpeedMapPassing.put(1.30,21.02);
        flywheelSpeedMapPassing.put(2.43, 24.0);
        flywheelSpeedMapPassing.put(3.04, 26.0);
        flywheelSpeedMapPassing.put(4.00, 27.5);
        flywheelSpeedMapPassing.put(5.5, 31.5);
        flywheelSpeedMapPassing.put(9.14, 38.0);
        flywheelSpeedMapPassing.put(12.95, 95.0);

        tofMapPassing.put(1.35, 1.00);
        tofMapPassing.put(2.43, 1.22);
        tofMapPassing.put(3.04, 1.28);
        tofMapPassing.put(3.5, 1.36);
        tofMapPassing.put(4.00, 1.33);
        tofMapPassing.put(5.5, 1.54);
        tofMapPassing.put(9.64, 1.67);
        tofMapPassing.put(13.95, 2.20);
    }

    public ShotData getData() {
        // Calculate estimated pose while accounting for time between calculation
        Pose2d estimatedRobotPose = poseSupplier.get();
        ChassisSpeeds robotRelativeVelocity = robotRelativeVelocitySupplier.get();
        ChassisSpeeds fieldRelativeVelocity = fieldVelocitySupplier.get();
        
        estimatedRobotPose = estimatedRobotPose.exp( // offset estimated robot pose to account for robot motion during processing delay
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));
        
        // Calculate distance from turret to target
        // Apply currently-configured target (default is the hub) with alliance flip
        Translation2d targetTranslation = AllianceFlipUtil.apply(this.target);
        // Decide whether to use hub calibration maps or passing maps. If the
        // configured target is one of the hubs (alliance or opponent) use the
        // hub maps; otherwise use the passing maps. Use a small distance
        // tolerance to account for construction differences.
        boolean isHubTarget = targetTranslation.getDistance(FieldConstants.Hub.topCenterPoint.toTranslation2d()) < 0.1
            || targetTranslation.getDistance(FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()) < 0.1;

        final InterpolatingTreeMap<Double, Rotation2d> activeHoodMap = isHubTarget ? hoodAngleMap : hoodAngleMapPassing;
        final InterpolatingDoubleTreeMap activeFlywheelMap = isHubTarget ? flywheelSpeedMap : flywheelSpeedMapPassing;
        final InterpolatingDoubleTreeMap activeTofMap = isHubTarget ? tofMap : tofMapPassing;
        // Use the configured robot->turret transform from ShooterConstants (drop Z)
        Translation3d robotToTurretTrans = ShooterConstants.kRobotToTurretTransform.getTranslation();
        Pose2d estimatedTurretPose = estimatedRobotPose.transformBy(
            new Transform2d(
                robotToTurretTrans.toTranslation2d(),
                new Rotation2d()));
        double turretToTargetDistance = targetTranslation.getDistance(estimatedTurretPose.getTranslation());

        // Calculate field relative turret velocity
        double robotAngle = estimatedRobotPose.getRotation().getRadians();

        // double dt = Constants.DriveConstants.kLoopPeriodSeconds;

        // ax = (robotVelocity.vxMetersPerSecond - lastFieldVelocity.vxMetersPerSecond) / dt;
        // ay = (robotVelocity.vyMetersPerSecond - lastFieldVelocity.vyMetersPerSecond) / dt;

        // lastFieldVelocity = robotVelocity;

        // v_robot + w x r
        // double turretVelocityX = robotVelocity.vxMetersPerSecond
        //     - robotVelocity.omegaRadiansPerSecond //maybe need to be +
        //         * (robotToTurretTrans.getX() * Math.sin(robotAngle)
        //         + robotToTurretTrans.getY() * Math.cos(robotAngle));

        // double turretVelocityY = robotVelocity.vyMetersPerSecond
        //     + robotVelocity.omegaRadiansPerSecond
        //         * (robotToTurretTrans.getX() * Math.cos(robotAngle)
        //         - robotToTurretTrans.getY() * Math.sin(robotAngle));

        double turretVelocityX = fieldRelativeVelocity.vxMetersPerSecond
            + fieldRelativeVelocity.omegaRadiansPerSecond
                * (robotToTurretTrans.getY() * Math.cos(robotAngle)
                    - robotToTurretTrans.getX() * Math.sin(robotAngle));
        double turretVelocityY =
        fieldRelativeVelocity.vyMetersPerSecond
            + fieldRelativeVelocity.omegaRadiansPerSecond
                * (robotToTurretTrans.getX() * Math.cos(robotAngle)
                    - robotToTurretTrans.getY() * Math.sin(robotAngle));

        double turretAccelX = ax;
        double turretAccelY = ay;

        // Account for imparted velocity by robot (turret) to offset
        double timeOfFlight;
        Pose2d lookaheadPose = estimatedTurretPose;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = activeTofMap.get(lookaheadTurretToTargetDistance);
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            // double offsetX = turretVelocityX * t + 0.5 * turretAccelX * t * t;
            // double offsetY = turretVelocityY * t + 0.5 * turretAccelY * t * t;
            lookaheadPose = new Pose2d(
                //CHANGE THIS IF LOOKAHEAD IS OPPOSITE
                estimatedTurretPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                estimatedTurretPose.getRotation());
            lookaheadTurretToTargetDistance = targetTranslation.getDistance(lookaheadPose.getTranslation());
        }

        // Calculate parameters accounted for imparted velocity
        // Get field-relative angle from turret to target
        // CHANGE THIS IF MOVING IN OPPOSITE DIRECTION
        double fieldRelativeAngleRad = targetTranslation.minus(lookaheadPose.getTranslation()).getAngle().getRadians();
        // Convert to robot-relative by subtracting robot heading
        double robotRelativeAngleRad = fieldRelativeAngleRad - estimatedRobotPose.getRotation().getRadians();
        // Normalize to [0, 2π]
        double rawTurretAngleRad = Math.atan2(Math.sin(robotRelativeAngleRad), Math.cos(robotRelativeAngleRad));
        rawTurretAngleRad = MathUtil.inputModulus(rawTurretAngleRad, ShooterConstants.turretMinAngle.get(), ShooterConstants.turretMaxAngle.get());

        // Filter the turret angle to smooth noisy measurements
        //double filteredTurretAngleRad = turretAngleFilter.calculate(rawTurretAngleRad);

        //turretAngle = Rotation2d.fromRadians(filteredTurretAngleRad);
        turretAngle = Rotation2d.fromRadians(rawTurretAngleRad);

        double flywheelSpeed = activeFlywheelMap.get(lookaheadTurretToTargetDistance) + velocityOffset.get();

        //Logger.recordOutput("ShotCalculator/Target", );


    hoodAngle = activeHoodMap.get(lookaheadTurretToTargetDistance).plus(Rotation2d.fromDegrees(hoodAngleOffset.get()));
        // Smooth hood angle as well
        hoodAngle = Rotation2d.fromRadians(hoodAngleFilter.calculate(hoodAngle.getRadians()));
    Logger.recordOutput("ShotCalculator/HoodAngle", activeHoodMap.get(lookaheadTurretToTargetDistance));
    Logger.recordOutput("ShotCalculator/isHubTarget", isHubTarget);


        if (lastTurretAngle == null)
            lastTurretAngle = turretAngle;
        if (lastHoodAngle == null)
            lastHoodAngle = hoodAngle;

        // Compute angular velocities (simple derivative on filtered angle)
        turretVelocity = (turretAngle.getRadians() - lastTurretAngle.getRadians()) / Constants.DriveConstants.kLoopPeriodSeconds;
        hoodVelocity = (hoodAngle.getRadians() - lastHoodAngle.getRadians()) / Constants.DriveConstants.kLoopPeriodSeconds;

        lastTurretAngle = turretAngle;
        lastHoodAngle = hoodAngle;
        // Valid only when distance in range AND turret angle is within +/- 3/4*pi
        latestData = new ShotData(
                true, //lookaheadTurretToTargetDistance >= minDistance && lookaheadTurretToTargetDistance <= maxDistance,
                //&& ((filteredTurretAngleRad) >= (1.0 / 2.0) * Math.PI && (filteredTurretAngleRad) <= (1.0/4.0) * Math.PI), // clamp to 180 degree max range (1/4 pi on each side)
                turretAngle,
                turretVelocity,
                hoodAngle,
                hoodVelocity,
                flywheelSpeed);

        // Log calculated values
        Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("LaunchCalculator/LookaheadTargetDistance", lookaheadTurretToTargetDistance);
        
        // Log calculated values for debugging
        Logger.recordOutput("ShotCalculator/RobotPose", estimatedRobotPose);
        Logger.recordOutput("ShotCalculator/TargetPos", targetTranslation);
        Logger.recordOutput("ShotCalculator/TurretPos", estimatedTurretPose.getTranslation());
        Logger.recordOutput("ShotCalculator/FieldRelativeAngle", fieldRelativeAngleRad);
        Logger.recordOutput("ShotCalculator/RobotRelativeAngle", robotRelativeAngleRad);
        Logger.recordOutput("ShotCalculator/RawTurretAngle", rawTurretAngleRad);
        //Logger.recordOutput("ShotCalculator/FilteredTurretAngle", filteredTurretAngleRad);
        Logger.recordOutput("ShotCalculator/FlywheelSpeed", flywheelSpeed);
        Logger.recordOutput("ShotCalculator/TargetDistance", turretToTargetDistance);
        Logger.recordOutput("ShotCalculator/AccelerationX", ax);
        Logger.recordOutput("ShotCalculator/AccelerationY", ay);


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

    public boolean shotValid() {
        return latestData != null && latestData.isValid;
    }
}