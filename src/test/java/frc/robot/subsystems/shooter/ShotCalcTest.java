package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.shooter.ShooterConstants.Calculator.ShotData;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
public class ShotCalcTest {

    @Test
    public void testCalculateTimeOfFlight_simple() {
        LinearVelocity v = MetersPerSecond.of(10.0);
        Angle angle = Radians.of(0.0);
        Distance d = Meters.of(10.0);
        Time t = ShotCalculator.calculateTimeOfFlight(v, angle, d);
        assertEquals(1.0, t.in(Seconds), 1e-9);
    }


    @Test
    public void testGetDistanceToTarget() {
        Pose2d robot = new Pose2d();
        Translation3d target = new Translation3d(3.0, 4.0, 1.0);
        Distance d = ShotCalculator.getDistanceToTarget(robot, target);
        // distance between (0,0) and (3,4) is 5 meters
        assertEquals(5.0, d.in(Meters), 1e-9);
    }

    @Test
    public void testCalculateAngleFromVelocity_matchesManualCalculation() {
        Pose2d robot = new Pose2d();
        Translation3d target = new Translation3d(10.0, 0.0, 2.0);
        LinearVelocity vel = MetersPerSecond.of(20.0);

        // replicate the internal calculation to compute expected angle
        double g = MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond);
        double v = vel.in(InchesPerSecond);
        double xDist = ShotCalculator.getDistanceToTarget(robot, target).in(Inches);
        double yDist = Meters.of(target.getZ()).in(Inches) - Meters.of(ShooterConstants.Calculator.kRobotToTurretTransform.getTranslation().getZ()).in(Inches);
        double expected = Math.atan(((v * v) + Math.sqrt(Math.pow(v, 4) - g * (g * xDist * xDist + 2 * yDist * v * v))) / (g * xDist));

        Angle calc = ShotCalculator.calculateAngleFromVelocity(robot, vel, target);
        assertEquals(expected, calc.in(Radians), 1e-9);
    }

    @Test
    public void testPredictTargetPos() {
        Translation3d target = new Translation3d(1.0, 2.0, 3.0);
        ChassisSpeeds speeds = new ChassisSpeeds(1.0, 2.0, 0.0);
        Time t = Seconds.of(2.0);
        Translation3d predicted = ShotCalculator.predictTargetPos(target, speeds, t);
        assertEquals(1.0 - 1.0 * 2.0, predicted.getX(), 1e-9);
        assertEquals(2.0 - 2.0 * 2.0, predicted.getY(), 1e-9);
        assertEquals(3.0, predicted.getZ(), 1e-9);
    }

    @Test
    public void testScaleLinearVelocity_matchesFormula() {
        Distance distance = Meters.of(2.0);
        double expected = ShooterConstants.Calculator.kBaseVel.in(InchesPerSecond)
            + ShooterConstants.Calculator.kVelMultiplier * Math.pow(distance.in(Inches), ShooterConstants.Calculator.kVelPower);
        LinearVelocity out = ShotCalculator.scaleLinearVelocity(distance);
        assertEquals(expected, out.in(InchesPerSecond), 1e-9);
    }

    @Test
    public void testCalculateShotFromFunnelClearance_basicSanity() {
        Pose2d robot = new Pose2d();
        Translation3d target = new Translation3d(5.0, 0.0, 1.2);
        ShotData sd = ShotCalculator.calculateShotFromFunnelClearance(robot, target, target);
        // basic sanity checks: finite, positive exit velocity and angle in [0,2pi)
        double v = sd.getExitVelocity().in(MetersPerSecond);
        double ang = sd.getHoodAngle().in(Radians);
        assertTrue(Double.isFinite(v) && v > 0);
        assertTrue(ang >= 0 && ang < 2.0 * Math.PI);
    }
    @Test
    public void testLinearAngularRoundTrip() {
        LinearVelocity v = MetersPerSecond.of(2.0);
        Distance r = Meters.of(0.05);
        AngularVelocity av = ShotCalculator.linearToAngularVelocity(v, r);
        LinearVelocity v2 = ShotCalculator.angularToLinearVelocity(av, r);
        assertEquals(v.in(MetersPerSecond), v2.in(MetersPerSecond), 1e-9);
    }

    @Test
    public void testCalculateHoodAngle_forward() {
        Pose2d robot = new Pose2d();
        // compute turret translation as used in calculateHoodAngle
        Pose3d pose3 = new Pose3d(robot);
        Translation3d turretPose3 = pose3.transformBy(ShooterConstants.Calculator.kRobotToTurretTransform).getTranslation();
        double turretX = turretPose3.getX();
        double turretY = turretPose3.getY();
        Translation3d target = new Translation3d(turretX + 5.0, turretY, turretPose3.getZ());
    Angle hood = ShotCalculator.calculateHoodAngle(robot, target);
    double actual = hood.in(Radians);
    // Print the value so the Gradle test runner shows it in console output for quick diagnosis
    System.out.println("ShotCalcTest: calculated hood angle (radians) = " + actual);
    // Accept either 0 or 2*pi since the implementation uses a [0,2pi) modulus
    double wrappedDelta = Math.min(Math.abs(actual - 0.0), Math.abs(actual - 2.0 * Math.PI));
    org.junit.jupiter.api.Assertions.assertTrue(wrappedDelta < 1e-6, () -> "hood angle not near 0 or 2pi: " + actual);
    }

    @Test
    public void testShotDataConstructorAndAccessors() {
        Translation3d target = new Translation3d(1.0, 2.0, 3.0);
        ShotData sd = new ShotData(MetersPerSecond.of(12.34), Radians.of(0.5), target);
        assertEquals(12.34, sd.getExitVelocity().in(MetersPerSecond), 1e-9);
        assertEquals(0.5, sd.getHoodAngle().in(Radians), 1e-9);
        assertEquals(target.getX(), sd.getTarget().getX(), 1e-9);
        assertEquals(target.getY(), sd.getTarget().getY(), 1e-9);
        assertEquals(target.getZ(), sd.getTarget().getZ(), 1e-9);
    }

    @Test
    public void testIterativeMovingShot_stationaryEqualsNonIterative() {
        Pose2d robot = new Pose2d();
        Translation3d target = new Translation3d(5.0, 0.0, 1.0);
        ChassisSpeeds zeroSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        ShotData base = ShotCalculator.calculateShotFromFunnelClearance(robot, target, target);
        ShotData iter = ShotCalculator.iterativeMovingShotFromFunnelClearance(robot, zeroSpeeds, target, 3);

        // With zero field speeds, iterative solution should match the non-iterative one
        assertEquals(base.getExitVelocity().in(MetersPerSecond), iter.getExitVelocity().in(MetersPerSecond), 1e-6);
        assertEquals(base.getHoodAngle().in(Radians), iter.getHoodAngle().in(Radians), 1e-6);
    }
}
