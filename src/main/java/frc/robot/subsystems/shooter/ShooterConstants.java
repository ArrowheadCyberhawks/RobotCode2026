package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Shooter constants and enums.
 */
public final class ShooterConstants {
    private ShooterConstants() {}

    // CAN IDs
    public static final int kFlywheelMotor1Id = 20;
    public static final int kHoodMotorId = 13;
    // Turret motors (merged into shooter)
    public static final int kTurnMotorId = 14;
    public static final int kFlywheelMotor2Id = 22;

    // Flywheel (velocity control)
    public static final double kFlywheelGearRatio = 1.0;
    public static final double kPFlywheel = 0.12;
    public static final double kIFlywheel = 0.0;
    public static final double kDFlywheel = 0.001;
    public static final double kVFlywheel = 0.12;
    public static final double kFlywheelShootRPM = 80.0;
    // Turret gear ratio (turn motor rotations per turret rotation)
    public static final double kTurretGearRatio = 1/12.8;
    // Turret PID (units: volts per radian)
    public static final double kPTurret = 1.0;
    public static final double kITurret = 0.0;
    public static final double kDTurret = 0.0;
    public static final double kTurretAllowedError = 0.02;
    // Maximum voltage to apply to turret motor
    public static final double kMaxTurretVolts = 6.0; 

    // === Hood (Motion Magic) ===
    public static final double kHoodGearRatio = 1/8.16;
    public static final double kHoodCruiseRps = 2.0;
    public static final double kHoodAccelRps2 = 4.0;
    public static final double kPHood = 0.5;
    public static final double kIHood = 0.0;
    public static final double kDHood = 0.0;
    public static final double kVHood = 0.0;
    public static final double kAHood = 0.0;
    public static final double kGHood = 0.4;
    public static final double kHoodAllowedError = 0.02;

    public static final double kHoodMinDegrees = 0.0;
    public static final double kHoodMaxDegrees = 70.0;

    // Default targets (used for SmartDashboard defaults)
    public static final double kShootFlywheelTarget = kFlywheelShootRPM;
    public static final double kShootHoodTarget = 35.0;

    public enum HoodPosition {
        STOW(Degrees.of(0.0)),
        LOW(Degrees.of(20.0)),
        MID(Degrees.of(35.0)),
        HIGH(Degrees.of(55.0));

        public final Angle angle;
        HoodPosition(Angle angle) { this.angle = angle; }
        public Angle getAngle() { return angle; }
        public double getDegrees() { return angle.in(Degrees); }
    }

    // Turret Motion Magic tuning (motor rotations/sec and rotations/sec^2)
    public static final double kTurretCruiseRps = 4.0;
    public static final double kTurretAccelRps2 = 8.0;

    /**
     * Constants used by the shooter calculator.
     */
    public static final class Calculator {
        private Calculator() {}

    // Base exit velocity for the flywheel (meters per second)
    public static final edu.wpi.first.units.measure.LinearVelocity kBaseVel =
        edu.wpi.first.units.Units.MetersPerSecond.of(100.0);

    // Multiplier and exponent applied to distance when scaling velocity
    public static final double kVelMultiplier = 0.01;
    public static final double kVelPower = 2.0;

    // Distance to clear the funnel above the funnel top (meters)
    public static final edu.wpi.first.units.measure.Distance kDistanceAboveFunnel =
        edu.wpi.first.units.Units.Inches.of(0.0);

    // Transform from robot pose to turret pose (default: identity)
    public static final edu.wpi.first.math.geometry.Transform3d kRobotToTurretTransform =
        new edu.wpi.first.math.geometry.Transform3d(new edu.wpi.first.math.geometry.Translation3d(), new edu.wpi.first.math.geometry.Rotation3d());
        
    // Lookahead iterations used by iterative shot prediction
    public static final int kLookaheadIterations = 3;

    // Radii used for converting linear exit velocity to angular wheel velocities
    // Default values (meters) - adjust to your hardware
    public static final edu.wpi.first.units.measure.Distance kFlywheelRadius = edu.wpi.first.units.Units.Meters.of(0.05);
    public static final edu.wpi.first.units.measure.Distance kShootRadius = edu.wpi.first.units.Units.Meters.of(0.05);

    public record ShotData(double exitVelocity, double hoodAngle, Translation3d target) {
        public ShotData(LinearVelocity exitVelocity, Angle hoodAngle, Translation3d target) {
            this(exitVelocity.in(MetersPerSecond), hoodAngle.in(Radians), target);
        }

        public LinearVelocity getExitVelocity() {
            return MetersPerSecond.of(this.exitVelocity);
        }

        public Angle getHoodAngle() {
            return Radians.of(this.hoodAngle);
        }

        public Translation3d getTarget() {
            return this.target;
        }
    }

    public static final InterpolatingDoubleTreeMap kDistanceToVelocity = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kDistanceToAngle = new InterpolatingDoubleTreeMap();


    static {
        // distance (meters) to m/s or radians
        // no, interpolating tree maps do not support using the units class directly, I tried (easier to do doubles anyways)
        kDistanceToVelocity.put(1.5, 12.0); // m/s
        kDistanceToAngle.put(1.5, 0.42); // radians
        kDistanceToVelocity.put(2.5, 14.0);
        kDistanceToAngle.put(2.5, 0.50);
    }

    }
}
