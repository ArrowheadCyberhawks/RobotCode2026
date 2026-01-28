package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.Calculator.ShotData;
import frc.robot.subsystems.shooter.ShooterConstants.HoodPosition;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import java.util.function.Supplier;
import static edu.wpi.first.units.Units.Rotations;
// import edu.wpi.first.units.measure.Voltage; // not needed; use VoltageOut constructor directly

//TODO: Add turret zeroing routine
//TODO: Only shoot when all components are at setpoint - but change tolerance based on distance so that due to angle the shot will be the same


public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX flywheel = new TalonFX(ShooterConstants.kFlywheelMotorId);
  private final TalonFX hood = new TalonFX(ShooterConstants.kHoodMotorId);
  // Turret motors (merged)
  private final TalonFX turnMotor = new TalonFX(ShooterConstants.kTurnMotorId);
  private final TalonFX shootMotor = new TalonFX(ShooterConstants.kShootMotorId);

  // Optional target supplier (pose of the target in field coordinates)
  private Supplier<Pose2d> targetPoseSupplier;
  private Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  // (Motion Magic will be used for turret control)

  private final VelocityVoltage flyRequest = new VelocityVoltage(0);
  private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);
  private final MotionMagicVoltage turretRequest = new MotionMagicVoltage(0);
  private final NeutralOut neutralOut = new NeutralOut();
  private final VoltageOut shootVoltageRequest = new VoltageOut(0);
  private final VelocityDutyCycle flywheelVelocityDuty = new VelocityDutyCycle(0);

  public ShooterSubsystem(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    configureFlywheel();
    configureHood();
    configureTurret();
    // Zero hood at known position (STOW)
    resetHoodEncoderToDegrees(HoodPosition.STOW.getDegrees());

    this.targetPoseSupplier = poseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;

    // Publish tunables to SmartDashboard
    SmartDashboard.putNumber("Shooter/Flywheel Target", ShooterConstants.kShootFlywheelTarget);
    SmartDashboard.putNumber("Shooter/Hood Target", ShooterConstants.kShootHoodTarget);

  // Turret Motion Magic tunables (exposed for convenience)
  SmartDashboard.putNumber("Shooter/Turret/MaxVolts", ShooterConstants.kMaxTurretVolts);
  }

  private void configureFlywheel() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.Slot0.kP = ShooterConstants.kPFlywheel;
    cfg.Slot0.kI = ShooterConstants.kIFlywheel;
    cfg.Slot0.kD = ShooterConstants.kDFlywheel;
    cfg.Slot0.kV = ShooterConstants.kVFlywheel;
    flywheel.getConfigurator().apply(cfg);
  }

  private void configureHood() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Slot0.kP = ShooterConstants.kPHood;
    cfg.Slot0.kI = ShooterConstants.kIHood;
    cfg.Slot0.kD = ShooterConstants.kDHood;
    cfg.Slot0.kV = ShooterConstants.kVHood;
    cfg.Slot0.kA = ShooterConstants.kAHood;
    cfg.Slot0.kG = ShooterConstants.kGHood;
    cfg.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kHoodCruiseRps;
    cfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.kHoodAccelRps2;
    hood.getConfigurator().apply(cfg);
  }

  /**
   * Set the shooter flywheel (secondary) wheel RPS.
   */
  public void setShootWheelRPS(double rps) {
    double motorRps = rps * ShooterConstants.kFlywheelGearRatio;
    shootMotor.setControl(flyRequest.withVelocity(motorRps));
  }

  /**
   * Aim and apply a shot using the ShooterCalculator. This computes the desired
   * turret azimuth, hood angle, and wheel speeds and applies them to the
   * hardware.
   * poseSupplier provides the robot Pose2d; fieldSpeedsSupplier provides current
   * field-relative speeds.
   */
  public void aimAndShoot(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    // Pick the alliance-specific hub target
    Translation3d currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? FieldConstants.HUB_BLUE
        : FieldConstants.HUB_RED;

    Pose2d robot = poseSupplier.get();
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

    ShotData calculatedShot = ShotCalculator.iterativeMovingShotFromFunnelClearance(
        robot, fieldSpeeds, currentTarget, ShooterConstants.Calculator.kLookaheadIterations);
    Angle hoodAngle = ShotCalculator.calculateHoodAngle(robot, calculatedShot.getTarget());
    double desiredAngle = hoodAngle.in(edu.wpi.first.units.Units.Radians);

    // Command turret with Motion Magic (convert desired turret radians -> motor rotations)
    moveTurretToRadians(desiredAngle);

    // Hood (convert to degrees)
    double hoodRad = calculatedShot.getHoodAngle().in(edu.wpi.first.units.Units.Radians);
    double hoodDeg = Math.toDegrees(hoodRad);
    moveHoodToDegrees(hoodDeg);

    // Flywheel
    AngularVelocity flyAng = ShotCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(),
        ShooterConstants.Calculator.kFlywheelRadius);
    double flyRps = flyAng.in(edu.wpi.first.units.Units.RadiansPerSecond) / (2.0 * Math.PI);
    setFlywheelRPS(flyRps);

    // Shooter secondary wheel
    AngularVelocity shootAng = ShotCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(),
        ShooterConstants.Calculator.kShootRadius);
    double shootRps = shootAng.in(edu.wpi.first.units.Units.RadiansPerSecond) / (2.0 * Math.PI);
    setShootWheelRPS(shootRps);
  }

  /**
   * Move turret to an absolute angle (radians) using Motion Magic.
   * @param radians desired turret angle (radians, field-relative as computed by calculator)
   */
  public void moveTurretToRadians(double radians) {

    //double turretRotations = shortestRad / (2.0 * Math.PI); //has wrapping

    // switch back to the earlier version when switching back over to talons
    // CTRE has a Continuous Mechanism Wrap closed-loop config, so we do not need to find the shortest path
    // This prevents commanding the long rotation across the 0/2pi boundary.
    double currentRad = getTurretRotation().getRadians();
    double rawDiff = radians - currentRad;
    // put in range of [-pi, pi]
    double delta = Math.atan2(Math.sin(rawDiff), Math.cos(rawDiff));
    double shortestRad = currentRad + delta;
    // Convert desired turret radians (shortest equivalent) to motor rotations
    double turretRotations = shortestRad / (2.0 * Math.PI);


    double motorRotations = turretRotations * ShooterConstants.kTurretGearRatio;
    turnMotor.setControl(turretRequest.withPosition(motorRotations));
  }

  public void moveTurretToDegrees(double degrees) {
    moveTurretToRadians(Math.toRadians(degrees));
  }

  // Flywheel control
  public void setFlywheelRPS(double rps) {
    double motorRps = rps * ShooterConstants.kFlywheelGearRatio;
    flywheel.setControl(flyRequest.withVelocity(motorRps));
  }

  public void stopFlywheel() {
    flywheel.setControl(neutralOut);
  }

  public double getFlywheelRPS() {
    return flywheel.getVelocity().getValueAsDouble() / ShooterConstants.kFlywheelGearRatio;
  }

  // Turret/turn control (merged)
  public void setTurretVoltage(double volts) {
    // Create a direct VoltageOut control with the desired voltage
    turnMotor.setControl(new VoltageOut(volts));
  }

  public void stopTurret() {
    turnMotor.setControl(neutralOut);
  }

  public void resetTurretEncoder() {
    double rot = turnMotor.getPosition().getValue().in(Rotations);
    turnMotor.setPosition(rot % 1.0);
  }

  public Rotation2d getTurretRotation() {
    try {
      double rotations = turnMotor.getPosition().getValue().in(Rotations);
      double turretRotations = rotations / ShooterConstants.kTurretGearRatio;
      return Rotation2d.fromRadians(turretRotations * 2.0 * Math.PI);
    } catch (Exception e) {
      return new Rotation2d();
    }
  }

  private void configureTurret() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.ClosedLoopGeneral.ContinuousWrap = true;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Slot0.kP = ShooterConstants.kPTurret;
    cfg.Slot0.kI = ShooterConstants.kITurret;
    cfg.Slot0.kD = ShooterConstants.kDTurret;
    // feedforward / velocity gains may be added later
    cfg.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.kTurretCruiseRps;
    cfg.MotionMagic.MotionMagicAcceleration = ShooterConstants.kTurretAccelRps2;
    turnMotor.getConfigurator().apply(cfg);
  }

  /**
   * Set the supplier that provides the target Pose2d (field coordinates).
   * If null, automatic aiming is disabled.
   * Should be used to set a target pose of corners for passing or goal for
   * shooting
   */
  public void setTargetPoseSupplier(Supplier<Pose2d> supplier) {
    this.targetPoseSupplier = supplier;
  }

  public void moveHoodTo(HoodPosition pos) {
    hood.setControl(hoodRequest.withPosition(degreesToMotorRotations(pos.getDegrees())));
  }

  public void moveHoodToDegrees(double degrees) {
    double clipped = Math.max(ShooterConstants.kHoodMinDegrees,
        Math.min(ShooterConstants.kHoodMaxDegrees, degrees));
    hood.setControl(hoodRequest.withPosition(degreesToMotorRotations(clipped)));
  }

  public void stopHood() {
    hood.setControl(neutralOut);
  }

  public double getHoodDegrees() {
    return motorRotationsToDegrees(hood.getPosition().getValueAsDouble());
  }

  public void resetHoodEncoderToDegrees(double degrees) {
    hood.setPosition(degreesToMotorRotations(degrees));
  }

  // Conversions
  private double degreesToMotorRotations(double degrees) {
    return (degrees / 360.0) * ShooterConstants.kHoodGearRatio;
  }

  private double motorRotationsToDegrees(double motorRot) {
    return (motorRot / ShooterConstants.kHoodGearRatio) * 360.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Flywheel RPS", getFlywheelRPS());
    SmartDashboard.putNumber("Shooter/Hood Degrees", getHoodDegrees());

    // Update PID tunables
  // (turret now uses Motion Magic; PID tunables removed)

    aimAndShoot(targetPoseSupplier, chassisSpeedsSupplier);
  }
}
