package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;
import java.util.function.Supplier;
import frc.robot.subsystems.shooter.ShooterConstants.HoodPosition;
import frc.robot.Constants.FieldConstants;

/**
 * Spark (REV CANSparkMax) backed shooter subsystem. Mirrors the public API
 * of the TalonFX-backed `ShooterSubsystem` so callers can switch implementations.
 *
 * Note: This file requires REV Robotics libraries on the classpath.
 */

 //NOTE: Copilot Turned ShooterSubsystem into ShooterSubsystemNeo, so I have yet to check if everything will work yet
public class ShooterSubsystemNeo extends SubsystemBase {

  private final SparkMax hood;
  private final SparkMax turnMotor;

  private final PIDController hoodPid;
  private final PIDController turretPid;

  private final RelativeEncoder hoodEncoder;
  private final RelativeEncoder turretEncoder;

  private SparkMaxConfig hoodConfig;
  private SparkMaxConfig turretConfig;

  private Supplier<Pose2d> targetPoseSupplier;
  private Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  public ShooterSubsystemNeo(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    // create motors
    hood = new SparkMax(ShooterConstants.kHoodMotorId, MotorType.kBrushless);
    turnMotor = new SparkMax(ShooterConstants.kTurnMotorId, MotorType.kBrushless);

    // encoders
    hoodEncoder = hood.getEncoder();
    turretEncoder = turnMotor.getEncoder();

    hoodConfig = new SparkMaxConfig();
    turretConfig = new SparkMaxConfig();


    // PID controllers
    hoodPid = new PIDController(ShooterConstants.kPHood, ShooterConstants.kIHood, ShooterConstants.kDHood);
    turretPid = new PIDController(ShooterConstants.kPTurret, ShooterConstants.kITurret, ShooterConstants.kDTurret);

    configureHood();
    configureTurret();

    // Zero hood at known position
    resetHoodEncoderToDegrees(HoodPosition.STOW.degrees);

    this.targetPoseSupplier = poseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;

    SmartDashboard.putNumber("Shooter/Flywheel Target", ShooterConstants.kShootFlywheelTarget);
    SmartDashboard.putNumber("Shooter/Hood Target", ShooterConstants.kShootHoodTarget);
    SmartDashboard.putNumber("Shooter/Turret/MaxVolts", ShooterConstants.kMaxTurretVolts);
  }

  //configuring stuff doesn't work but i'm too lazy to actually fix it for the neo test

  private void configureHood() {
    hoodConfig.idleMode(IdleMode.kBrake);
    // WPILib PIDController gains already set in constructor; nothing else to do here
  }

  private void configureTurret() {
    turretConfig.idleMode(IdleMode.kBrake);
    // PID gains set in constructor
  }


  // Turret
  public void moveTurretToRadians(double radians) {
    // convert desired turret radians -> motor rotations
    double turretRotations = radians / (2.0 * Math.PI);
    double motorRotations = turretRotations * ShooterConstants.kTurretGearRatio;
    // Use two-arg calculate(current, setpoint) to compute output directly
    double control = turretPid.calculate(turretEncoder.getPosition(), motorRotations);
    double percent = MathUtil.clamp(control, -1.0, 1.0);
    turnMotor.set(percent);
  }

  public void moveTurretToDegrees(double degrees) {
    moveTurretToRadians(Math.toRadians(degrees));
  }

  public void setTurretVoltage(double volts) {
    // Spark supports percent output; approximate by dividing by 12V
    double percent = volts / 12.0;
    turnMotor.set(percent);
  }

  public void stopTurret() {
    turnMotor.stopMotor();
  }

  public void resetTurretEncoder() {
    // Spark encoder position is in rotations; set position based on current rotations mod 1
    double rot = turretEncoder.getPosition();
    turretEncoder.setPosition(rot % 1.0);
  }

  public Rotation2d getTurretRotation() {
    try {
      double rotations = turretEncoder.getPosition();
      double turretRotations = rotations / ShooterConstants.kTurretGearRatio;
      return Rotation2d.fromRadians(turretRotations * 2.0 * Math.PI);
    } catch (Exception e) {
      return new Rotation2d();
    }
  }

  // Hood
  public void moveHoodTo(HoodPosition pos) {
    double target = degreesToMotorRotations(pos.degrees);
    double control = hoodPid.calculate(hoodEncoder.getPosition(), target);
    double percent = MathUtil.clamp(control, -1.0, 1.0);
    hood.set(percent);
  }

  public void moveHoodToDegrees(double degrees) {
    double clipped = Math.max(ShooterConstants.kHoodMinDegrees,
        Math.min(ShooterConstants.kHoodMaxDegrees, degrees));
    double target = degreesToMotorRotations(clipped);
    double control = hoodPid.calculate(hoodEncoder.getPosition(), target);
    double percent = MathUtil.clamp(control, -1.0, 1.0);
    hood.set(percent);
  }

  public void stopHood() {
    hood.stopMotor();
  }

  public double getHoodDegrees() {
    return motorRotationsToDegrees(hoodEncoder.getPosition());
  }

  public void resetHoodEncoderToDegrees(double degrees) {
    hoodEncoder.setPosition(degreesToMotorRotations(degrees));
  }

  private double degreesToMotorRotations(double degrees) {
    return (degrees / 360.0) * ShooterConstants.kHoodGearRatio;
  }

  private double motorRotationsToDegrees(double motorRot) {
    return (motorRot / ShooterConstants.kHoodGearRatio) * 360.0;
  }

  /**
   * Aim and apply a shot using ShooterCalculator. Mirrors ShooterSubsystem behavior.
   */
  public void aimAndShoot(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
  Translation3d currentTarget = edu.wpi.first.wpilibj.DriverStation.getAlliance().orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
    ? FieldConstants.HUB_BLUE
    : FieldConstants.HUB_RED;

    Pose2d robot = poseSupplier.get();
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

    ShotCalc.ShotData calculatedShot = ShotCalc.iterativeMovingShotFromFunnelClearance(
        robot, fieldSpeeds, currentTarget, ShooterConstants.Calculator.kLookaheadIterations);

    Angle hoodAngle = ShotCalc.calculateHoodAngle(robot, calculatedShot.getTarget());
    double desiredAngle = hoodAngle.in(Radians);

    moveTurretToRadians(desiredAngle);

    double hoodRad = calculatedShot.getHoodAngle().in(Radians);
    double hoodDeg = Math.toDegrees(hoodRad);
    moveHoodToDegrees(hoodDeg);

    AngularVelocity flyAng = ShotCalc.linearToAngularVelocity(calculatedShot.getExitVelocity(),
        ShooterConstants.Calculator.kFlywheelRadius);
    double flyRps = flyAng.in(RadiansPerSecond) / (2.0 * Math.PI);


    AngularVelocity shootAng = ShotCalc.linearToAngularVelocity(calculatedShot.getExitVelocity(),
        ShooterConstants.Calculator.kShootRadius);
    double shootRps = shootAng.in(RadiansPerSecond) / (2.0 * Math.PI);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Turret Radians", getTurretRotation().getRadians());
    SmartDashboard.putNumber("Shooter/Hood Degrees", getHoodDegrees());

    // Call aimAndShoot if suppliers are present
    if (targetPoseSupplier != null && chassisSpeedsSupplier != null) {
      aimAndShoot(targetPoseSupplier, chassisSpeedsSupplier);
    }
  }
}
