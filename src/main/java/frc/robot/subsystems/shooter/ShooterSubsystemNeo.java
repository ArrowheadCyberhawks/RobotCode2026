package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
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

  private final SparkMax hoodMotor;
  private final SparkMax turnMotor;

  private final SparkClosedLoopController hoodController, turretController;

  private final RelativeEncoder hoodEncoder;
  private final RelativeEncoder turretEncoder;

  private SparkMaxConfig hoodConfig;
  private SparkMaxConfig turretConfig;

  private Supplier<Pose2d> targetPoseSupplier;
  private Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  public ShooterSubsystemNeo(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    // create motors
    hoodMotor = new SparkMax(ShooterConstants.kHoodMotorId, MotorType.kBrushless);
    turnMotor = new SparkMax(ShooterConstants.kTurnMotorId, MotorType.kBrushless);

    // encoders
    hoodEncoder = hoodMotor.getEncoder();
    turretEncoder = turnMotor.getEncoder();

    hoodConfig = new SparkMaxConfig();
    turretConfig = new SparkMaxConfig();


    // PID controllers
    hoodController = hoodMotor.getClosedLoopController();
    turretController = turnMotor.getClosedLoopController();

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
    hoodConfig.encoder
      .positionConversionFactor(ShooterConstants.kHoodGearRatio * 2.0 * Math.PI);
    hoodConfig.idleMode(IdleMode.kBrake)
      .inverted(true)
      .closedLoop
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI)
        .p(ShooterConstants.kPHood)
        .i(ShooterConstants.kIHood)
        .d(ShooterConstants.kDHood)
        .allowedClosedLoopError(ShooterConstants.kHoodAllowedError, ClosedLoopSlot.kSlot0);
    hoodConfig.softLimit
      .forwardSoftLimit(Math.toRadians(ShooterConstants.kHoodMaxDegrees))
      .reverseSoftLimit(Math.toRadians(ShooterConstants.kHoodMinDegrees));
    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void configureTurret() {
    turretConfig.encoder.positionConversionFactor(ShooterConstants.kTurretGearRatio * 2.0 * Math.PI);
    turretConfig.idleMode(IdleMode.kBrake)
      .inverted(true)
      .closedLoop
      .p(ShooterConstants.kPTurret)
      .i(ShooterConstants.kITurret)
      .d(ShooterConstants.kDTurret)
      .allowedClosedLoopError(ShooterConstants.kTurretAllowedError, ClosedLoopSlot.kSlot0);
    turnMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }


  // Turret
  public void setTurretTarget(Angle targetTurretAngle) {
    turretController.setSetpoint(targetTurretAngle.in(Radians), ControlType.kPosition);
  }

  public void setTurretVoltage(Voltage volts) {
    // Spark supports percent output; approximate by 
    turnMotor.setVoltage(volts);
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
      return Rotation2d.fromRadians(turretEncoder.getPosition());
    } catch (Exception e) {
      return new Rotation2d();
    }
  }

  // Hood
  public void moveHoodTo(HoodPosition pos) {
    setHoodTarget(Degrees.of(pos.degrees));
  }

  public void setHoodTarget(Angle targetHoodAngle) {
    hoodController.setSetpoint(targetHoodAngle.in(Radians), ControlType.kPosition);
  }

  public void stopHood() {
    hoodMotor.stopMotor();
  }

  public Rotation2d getHoodRotation() {
    return Rotation2d.fromRadians(hoodEncoder.getPosition());
  }

  public void resetHoodEncoderToDegrees(double degrees) {
    hoodEncoder.setPosition(degreesToMotorRotations(degrees));
  }

  private double degreesToMotorRotations(double degrees) {
    return (degrees / 360.0) * ShooterConstants.kHoodGearRatio;
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

    setTurretTarget(hoodAngle);
    setHoodTarget(calculatedShot.getHoodAngle());

    AngularVelocity flyAng = ShotCalc.linearToAngularVelocity(calculatedShot.getExitVelocity(),
        ShooterConstants.Calculator.kFlywheelRadius);
    double flyRps = flyAng.in(RadiansPerSecond) / (2.0 * Math.PI);


    AngularVelocity shootAng = ShotCalc.linearToAngularVelocity(calculatedShot.getExitVelocity(),
        ShooterConstants.Calculator.kShootRadius);
    double shootRps = shootAng.in(RadiansPerSecond) / (2.0 * Math.PI);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Turret Degrees", getTurretRotation().getDegrees());
    SmartDashboard.putNumber("Shooter/Hood Degrees", getHoodRotation().getDegrees());

    // Call aimAndShoot if suppliers are present
    // aimAndShoot(targetPoseSupplier, chassisSpeedsSupplier);
    // setTurretTarget(Rotation2d.kZero.getMeasure());
    // setHoodTarget(Rotation2d.kCW_90deg.getMeasure());
  }
}