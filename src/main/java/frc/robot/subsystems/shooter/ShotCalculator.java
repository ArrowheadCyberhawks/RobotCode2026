package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import frc.robot.subsystems.shooter.ShooterConstants.Calculator.ShotData;
import static frc.robot.subsystems.shooter.ShooterConstants.Calculator.kDistanceToVelocity;
import static frc.robot.subsystems.shooter.ShooterConstants.Calculator.kDistanceToAngle;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.FieldConstants;

// Possibly abandon this model completely and just use it for calculating whether to take the high or low shot

//TODO: Account for air drag (non-constant x velocity)
//TODO: Account gravity for magnus effect (the used value of g is not equal to 9.81 m/s^2 when the ball is spinning)

public class ShotCalculator {
	public static Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
		return Meters.of(robot.getTranslation().getDistance(target.toTranslation2d()));
	}

	public static Angle calculateAngleFromVelocity(Pose2d robot, LinearVelocity velocity, Translation3d target) {
		double g = MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond); // gravity!
		double vel = velocity.in(InchesPerSecond);
		double xDist = getDistanceToTarget(robot, target).in(Inches);
		double yDist = Meters.of(target.getZ()).in(Inches)
				- Meters.of(ShooterConstants.Calculator.kRobotToTurretTransform.getTranslation().getZ()).in(Inches);
		// derived from projectile motion equations, substitute solve for t using x(t),
		// then plug into y and solve for theta
		double angle = Math
				.atan(((vel * vel) + Math.sqrt(Math.pow(vel, 4) - g * (g * xDist * xDist + 2 * yDist * vel * vel)))
						/ (g * xDist));
		return Radians.of(angle);
	}

	// calculates how long it will take for a projectile to travel a set distance
	// given its initial velocity and angle
	// TODO: add air drag into TOF equation
	public static Time calculateTimeOfFlight(LinearVelocity exitVelocity, Angle hoodAngle, Distance distance) {
		double vel = exitVelocity.in(MetersPerSecond);
		double angle = hoodAngle.in(Radians);
		double dist = distance.in(Meters);
		return Seconds.of(dist / (vel * Math.cos(angle)));
	}

	public static AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
		return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters));
	}

	public static LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
		return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters));
	}

	// calculates the angle of a turret relative to the robot to hit a target
	public static Angle calculateHoodAngle(Pose2d robot, Translation3d target) {
		Translation2d turretTranslation = new Pose3d(robot)
				.transformBy(ShooterConstants.Calculator.kRobotToTurretTransform)
				.toPose2d()
				.getTranslation();

		Translation2d direction = target.toTranslation2d().minus(turretTranslation);

		return direction.getAngle().minus(robot.getRotation()).getMeasure();
	}

	// Move a target a set time in the future along a velocity defined by
	// fieldSpeeds
	public static Translation3d predictTargetPos(Translation3d target, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
		double predictedX = target.getX() - fieldSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
		double predictedY = target.getY() - fieldSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);

		return new Translation3d(predictedX, predictedY, target.getZ());
	}

	// Custom velocity ramp meant to minimize how fast the flywheels have to change
	// speed
	public static LinearVelocity scaleLinearVelocity(Distance distanceToTarget) {
		double velocity = ShooterConstants.Calculator.kBaseVel.in(InchesPerSecond)
				+ ShooterConstants.Calculator.kVelMultiplier * Math.pow(distanceToTarget.in(Inches),
						ShooterConstants.Calculator.kVelPower);
		return InchesPerSecond.of(velocity);
	}

	public static ShotData calculateShotFromFunnelClearance(Pose2d robot, Translation3d actualTarget,
			Translation3d predictedTarget) {
		double xDist = getDistanceToTarget(robot, predictedTarget).in(Inches);
		double yDist = Meters.of(predictedTarget.getZ()).in(Inches)
				- Meters.of(ShooterConstants.Calculator.kRobotToTurretTransform.getTranslation().getZ()).in(Inches);
		double g = 386;
		double r = FieldConstants.FUNNEL_RADIUS.in(Inches) * xDist
				/ getDistanceToTarget(robot, actualTarget).in(Inches);
		double h = FieldConstants.FUNNEL_HEIGHT.plus(ShooterConstants.Calculator.kDistanceAboveFunnel).in(Inches);
		double A1 = xDist * xDist;
		double B1 = xDist;
		double D1 = yDist;
		double A2 = -xDist * xDist + (xDist - r) * (xDist - r);
		double B2 = -r;
		double D2 = h;
		double Bm = -B2 / B1;
		double A3 = Bm * A1 + A2;
		double D3 = Bm * D1 + D2;
		double a = D3 / A3;
		double b = (D1 - A1 * a) / B1;
		double theta = Math.atan(b);
		double v0 = Math.sqrt(-g / (2 * a * (Math.cos(theta)) * (Math.cos(theta))));
		return new ShotData(InchesPerSecond.of(v0), Radians.of(theta), predictedTarget);
	}

	// use an iterative lookahead approach to determine shot parameters for a moving
	// robot
	public static ShotData iterativeMovingShotFromFunnelClearance(Pose2d robot, ChassisSpeeds fieldSpeeds,
			Translation3d target, int iterations) {
		// Perform initial estimation (assuming unmoving robot) to get time of flight
		// estimate
		ShotData shot = calculateShotFromFunnelClearance(robot, target, target);
		Distance distance = getDistanceToTarget(robot, target);
		Time timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(), distance);
		Translation3d predictedTarget = target;

		// Iterate the process, getting better time of flight estimations and updating
		// the predicted target accordingly
		for (int i = 0; i < iterations; i++) {
			predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
			shot = calculateShotFromFunnelClearance(robot, target, predictedTarget);
			timeOfFlight = calculateTimeOfFlight(shot.getExitVelocity(), shot.getHoodAngle(),
					getDistanceToTarget(robot, predictedTarget));
		}

		return shot;
	}

	// We could shift to this to remove on-robot calculations and use raw heuristic
	// data
	// TODO: Prepare a second version of the shooter calculations using an
	// InterpolatingDoubleTreeMap in case the physics descrepancies are too much
	public static ShotData calculateHybridShot(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d target,
			int iterations) {
		// distance to target
		Distance baseDistance = getDistanceToTarget(robotPose, target);
		double baseDistanceMeters = baseDistance.in(Meters);

		// interpolated stationary shot
		double baseVelMetersPerSecond = kDistanceToVelocity.get(baseDistanceMeters);
		double baseAngleRadians = kDistanceToAngle.get(baseDistanceMeters);

		LinearVelocity exitVelocity = MetersPerSecond.of(baseVelMetersPerSecond);
		Angle hoodAngle = Radians.of(baseAngleRadians);

		// initial time of flight estimate
		Time timeOfFlight = calculateTimeOfFlight(exitVelocity, hoodAngle, baseDistance);

		Translation3d predictedTarget = target;

		// iternate with corrected distances
		for (int i = 0; i < iterations; i++) {
			// Predict where the target appears relative to robot motion
			predictedTarget = predictTargetPos(target, fieldSpeeds, timeOfFlight);
			Distance correctedDistance = getDistanceToTarget(robotPose, predictedTarget);
			double correctedMeters = correctedDistance.in(Meters);

			// Now interpolation using predicted distance
			double correctedVel = kDistanceToVelocity.get(correctedMeters);
			double correctedAngle = kDistanceToAngle.get(correctedMeters);

			exitVelocity = MetersPerSecond.of(correctedVel);
			hoodAngle = Radians.of(correctedAngle);

			// Update time of flight with corrected values
			timeOfFlight = calculateTimeOfFlight(exitVelocity, hoodAngle, correctedDistance);
		}

		return new ShotData(exitVelocity, hoodAngle, predictedTarget);
	}

}
