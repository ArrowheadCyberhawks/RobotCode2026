package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.LoggedTunableNumber;

/**
 * Shooter constants and enums.
 */
public final class ShooterConstants {
    public static final LoggedTunableNumber kMaxTurretAcceleration = new LoggedTunableNumber("Turret/MaxAcceleration", 10.0);
    public static final LoggedTunableNumber kMaxTurretVelocity = new LoggedTunableNumber("Turret/MaxVelocity", 10.0);

    private ShooterConstants() {}

    // CAN IDs
    public static final int kFlywheelMotorId = 16;
    public static final int kFlywheelFollowerMotorId = 17;
    public static final int kHoodMotorId = 10;
    public static final int kTurretMotorId = 13;

    // Flywheel (velocity control)
    public static final double kFlywheelGearRatio = 1.0;
    public static final LoggedTunableNumber kPFlywheel = new LoggedTunableNumber("Flywheel/kP", 0.20);
    public static final LoggedTunableNumber kIFlywheel = new LoggedTunableNumber("Flywheel/kI", 0.0);
    public static final LoggedTunableNumber kDFlywheel = new LoggedTunableNumber("Flywheel/kD", 0.02);
    public static final LoggedTunableNumber kVFlywheel = new LoggedTunableNumber("Flywheel/kV", 0.12);
    public static final LoggedTunableNumber kSFlywheel = new LoggedTunableNumber("Flywheel/kS", 0.25);

    public static final AngularVelocity kFlywheelMaxVel = RotationsPerSecond.of(55.0);
    public static final AngularVelocity kFlywheelMinVel = RotationsPerSecond.of(10.0);
    // Turret gear ratio (turret motor rotations per turret rotation)
    public static final double kTurretGearRatio = 1/5.0 * 20.0/100.0;
    
    // Turret PID (units: volts per radian)
    public static final LoggedTunableNumber kPTurret = new LoggedTunableNumber("Turret/kP", 5.0);
    public static final LoggedTunableNumber kITurret = new LoggedTunableNumber("Turret/kI", 0.0);
    public static final LoggedTunableNumber kDTurret = new LoggedTunableNumber("Turret/kD", 0.1);
    public static final LoggedTunableNumber kSTurret = new LoggedTunableNumber("Turret/kS", 0.0);
    public static final LoggedTunableNumber kVTurret = new LoggedTunableNumber("Turret/kV", 0.0);
    public static final LoggedTunableNumber kATurret = new LoggedTunableNumber("Turret/kA", 0.0);
    public static final LoggedTunableNumber turretTolerance = new LoggedTunableNumber("Turret/Tolerance", 0.05);
	public static final LoggedTunableNumber turretMaxPercentOutput = new LoggedTunableNumber("Turret/MaxPercentOutput", 0.60);
	public static final LoggedTunableNumber turretMaxAngle = new LoggedTunableNumber("Turret/MaxAngle", 7 * Math.PI / 4);
	public static final LoggedTunableNumber turretMinAngle = new LoggedTunableNumber("Turret/MinAngle", -Math.PI / 4); //TODO: move these into shooterconstants

    // Hood (Motion Magic)
    public static final double kHoodGearRatio = 1/4.0 * 1/4.0 * 30.0/364.0;
    public static final double kHoodCruiseRps = 2.0;
    public static final double kHoodAccelRps2 = 4.0;
    public static final LoggedTunableNumber kPHood = new LoggedTunableNumber("Hood/kP", 25);
    public static final LoggedTunableNumber kIHood = new LoggedTunableNumber("Hood/kI", 0.0);
    public static final LoggedTunableNumber kDHood = new LoggedTunableNumber("Hood/kD", 0.0);
    public static final LoggedTunableNumber kVHood = new LoggedTunableNumber("Hood/kV", 0.0);
    public static final LoggedTunableNumber kAHood = new LoggedTunableNumber("Hood/kA", 0.0);
    public static final LoggedTunableNumber kGHood = new LoggedTunableNumber("Hood/kG", 0.0);
    public static final double kHoodAllowedError = 1.0;

    public static final double kHoodMinDegrees = 15.0;
    public static final double kHoodMaxDegrees = 45.0;

    /** Common hood presets */
    public enum HoodPosition {
        STOW(kHoodMinDegrees);

        private final double degrees;

        HoodPosition(double degrees) {
            this.degrees = degrees;
        }

        public double getDegrees() {
            return degrees;
        }

        public Rotation2d getRotation() {
            return Rotation2d.fromDegrees(degrees);
        }
    }

    // Default targets
    public static final AngularVelocity kShootFlywheelTarget = kFlywheelMaxVel;
    public static final double kShootHoodTarget = 35.0;

    // Turret Motion Magic tuning (motor rotations/sec and rotations/sec^2)
    public static final double kTurretCruiseRps = 4.0;
    public static final double kTurretAccelRps2 = 8.0;

    // Transform from robot pose to turret pose
    public static final Transform3d kRobotToTurretTransform =
        new Transform3d(new Translation3d(-0.1473, 0.0907, 0.0), new Rotation3d());
        
    public static final Translation3d robotToTurret = kRobotToTurretTransform.getTranslation();
}
