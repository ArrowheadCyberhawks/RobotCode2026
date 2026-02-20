package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.LoggedTunableNumber;

/**
 * Shooter constants and enums.
 */
public final class ShooterConstants {
    private ShooterConstants() {}

    // CAN IDs
    public static final int kFlywheelMotorId = 16;
    public static final int kFlywheelFollowerMotorId = 17;
    public static final int kHoodMotorId = 10;
    public static final int kTurretMotorId = 13;

    // Flywheel (velocity control)
    public static final double kFlywheelGearRatio = 1.0;
    public static final LoggedTunableNumber kPFlywheel = new LoggedTunableNumber("Flywheel/kP", 0.0002);
    public static final LoggedTunableNumber kIFlywheel = new LoggedTunableNumber("Flywheel/kI", 0.0);
    public static final LoggedTunableNumber kDFlywheel = new LoggedTunableNumber("Flywheel/kD", 0.0);
    public static final LoggedTunableNumber kVFlywheel = new LoggedTunableNumber("Flywheel/kV", 0.002);
    public static final LoggedTunableNumber kSFlywheel = new LoggedTunableNumber("Flywheel/kS", 0.3);

    public static final double kFlywheelRpsShoot = 80.0;
    // Turret gear ratio (turret motor rotations per turret rotation)
    public static final double kTurretGearRatio = 0.02;
    
    // Turret PID (units: volts per radian)
    public static final LoggedTunableNumber kPTurret = new LoggedTunableNumber("Turret/kP", 1.9);
    public static final LoggedTunableNumber kITurret = new LoggedTunableNumber("Turret/kI", 0.0);
    public static final LoggedTunableNumber kDTurret = new LoggedTunableNumber("Turret/kD", 0.0);
    public static final LoggedTunableNumber kSTurret = new LoggedTunableNumber("Turret/kS", 0.0);
    public static final LoggedTunableNumber kVTurret = new LoggedTunableNumber("Turret/kV", 0.0);
    public static final LoggedTunableNumber kATurret = new LoggedTunableNumber("Turret/kA", 0.0);
    public static final double kTurretAllowedError = 0.02;
    // Maximum voltage to apply to turret motor
    public static final double kMaxTurretVolts = 6.0; 

    // Hood (Motion Magic)
    public static final double kHoodGearRatio = 1/8.16;
    public static final double kHoodCruiseRps = 2.0;
    public static final double kHoodAccelRps2 = 4.0;
    public static final LoggedTunableNumber kPHood = new LoggedTunableNumber("Hood/kP", 1.0);
    public static final LoggedTunableNumber kIHood = new LoggedTunableNumber("Hood/kI", 0.0);
    public static final LoggedTunableNumber kDHood = new LoggedTunableNumber("Hood/kD", 0.0);
    public static final LoggedTunableNumber kVHood = new LoggedTunableNumber("Hood/kV", 0.0);
    public static final LoggedTunableNumber kAHood = new LoggedTunableNumber("Hood/kA", 0.0);
    public static final LoggedTunableNumber kGHood = new LoggedTunableNumber("Hood/kG", 0.4);
    public static final double kHoodAllowedError = 0.02;

    public static final double kHoodMinDegrees = 0.0;
    public static final double kHoodMaxDegrees = 70.0;

    /** Common hood presets */
    public enum HoodPosition {
        STOW(20.0);

        private final double degrees;

        HoodPosition(double degrees) {
            this.degrees = degrees;
        }

        public double getDegrees() {
            return degrees;
        }

        public Angle getAngle() {
            return Degrees.of(degrees);
        }
    }

    // Default targets (used for SmartDashboard defaults)
    public static final double kShootFlywheelTarget = kFlywheelRpsShoot;
    public static final double kShootHoodTarget = 35.0;

    // Turret Motion Magic tuning (motor rotations/sec and rotations/sec^2)
    public static final double kTurretCruiseRps = 4.0;
    public static final double kTurretAccelRps2 = 8.0;

    // Transform from robot pose to turret pose (default: identity)
    public static final Transform3d kRobotToTurretTransform =
        new Transform3d(new Translation3d(), new Rotation3d());
        
    public static final Translation3d robotToTurret = kRobotToTurretTransform.getTranslation();
}
