package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.intake.IntakeConstants.IntakePosition;

/**
 * Intake subsystem using REV Spark Flex motor controllers.
 * Supports both simple PID position control and Smart Motion for the pivot.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex pivotMotor;
    private final SparkFlex rollerMotor;

    private final SparkClosedLoopController pivotController;
    private final SparkClosedLoopController rollerController;

    private final RelativeEncoder pivotEncoder;
    private final RelativeEncoder rollerEncoder;

    private SparkFlexConfig pivotConfig;
    private SparkFlexConfig rollerConfig;

    public IntakeSubsystem() {
        // Create motors
        pivotMotor = new SparkFlex(IntakeConstants.kPivotMotorId, MotorType.kBrushless);
        rollerMotor = new SparkFlex(IntakeConstants.kRollerMotorId, MotorType.kBrushless);

        // Get encoders
        pivotEncoder = pivotMotor.getEncoder();
        rollerEncoder = rollerMotor.getEncoder();

        // Create configs
        pivotConfig = new SparkFlexConfig();
        rollerConfig = new SparkFlexConfig();

        // Get closed loop controllers
        pivotController = pivotMotor.getClosedLoopController();
        rollerController = rollerMotor.getClosedLoopController();

        // Configure motors
        configurePivot();
        configureRoller();

        // Zero pivot at known position (stowed = 0 degrees)
        resetPivotEncoderToDegrees(0.0);

        SmartDashboard.putBoolean("Intake/UseSmartMotion", IntakeConstants.kUseSmartMotion);
    }

    private void configurePivot() {
        // Encoder conversion: motor rotations → radians of pivot
        pivotConfig.encoder
            .positionConversionFactor(IntakeConstants.kPivotGearRatio * 2.0 * Math.PI) // motor rotations to pivot radians
            .velocityConversionFactor(IntakeConstants.kPivotGearRatio * 2.0 * Math.PI / 60.0); // motor RPM to pivot radians per second

        // Motor output settings
        pivotConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);

        // Soft limits (converted to radians)
        pivotConfig.softLimit
            .forwardSoftLimit(Math.toRadians(IntakeConstants.kPivotMaxDegrees))
            .reverseSoftLimit(Math.toRadians(IntakeConstants.kPivotMinDegrees))
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        // PID configuration
        pivotConfig.closedLoop
            .p(IntakeConstants.kPPivot.get(), ClosedLoopSlot.kSlot0)
            .i(IntakeConstants.kIPivot.get(), ClosedLoopSlot.kSlot0)
            .d(IntakeConstants.kDPivot.get(), ClosedLoopSlot.kSlot0)
            .outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot0)
            .allowedClosedLoopError(IntakeConstants.kPivotToleranceRadians.get(), ClosedLoopSlot.kSlot0);

        // Smart Motion configuration
        pivotConfig.closedLoop
            .maxMotion
                .maxVelocity(IntakeConstants.kPivotMaxVelocityRps.get(), ClosedLoopSlot.kSlot0)
                .maxAcceleration(IntakeConstants.kPivotMaxAccelRps2.get(), ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(IntakeConstants.kPivotToleranceRadians.get(), ClosedLoopSlot.kSlot0);

        // Apply configuration
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void configureRoller() {
        // Roller doesn't need position conversion, just velocity (RPM default is fine)
        rollerConfig.encoder
            .velocityConversionFactor(1.0); // Keep as RPM, or convert if needed

        // Motor output settings
        rollerConfig
            .idleMode(IdleMode.kCoast)
            .inverted(false);

        // Velocity PID
        rollerConfig.closedLoop
            .p(IntakeConstants.kPRoller.get(), ClosedLoopSlot.kSlot0)
            .i(IntakeConstants.kIRoller.get(), ClosedLoopSlot.kSlot0)
            .d(IntakeConstants.kDRoller.get(), ClosedLoopSlot.kSlot0)
            .velocityFF(IntakeConstants.kVRoller.get(), ClosedLoopSlot.kSlot0)
            .outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot0);

        // Apply configuration
        rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // === Pivot Control Methods ===

    /**
     * Move the pivot to a predefined position.
     */
    public void moveToPosition(IntakePosition position) {
        setPivotTarget(position.getAngle());
    }

    /**
     * Set the pivot to a specific angle using either Smart Motion or simple PID.
     */
    public void setPivotTarget(Angle targetAngle) {
        double targetRadians = targetAngle.in(Radians);
        
        if (IntakeConstants.kUseSmartMotion) {
            pivotController.setReference(targetRadians, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        } else {
            pivotController.setReference(targetRadians, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
    }

    /**
     * Stop the pivot motor.
     */
    public void stopPivot() {
        pivotMotor.stopMotor();
    }

    /**
     * Get the current pivot angle.
     */
    public Rotation2d getPivotRotation() {
        return Rotation2d.fromRadians(pivotEncoder.getPosition());
    }

    /**
     * Get the current pivot angle in degrees.
     */
    public double getPivotDegrees() {
        return getPivotRotation().getDegrees();
    }

    /**
     * Reset the pivot encoder to a specific angle in degrees.
     */
    public void resetPivotEncoderToDegrees(double degrees) {
        double radians = Math.toRadians(degrees);
        pivotEncoder.setPosition(radians);
    }

    /**
     * Check if the pivot is at the target position (within tolerance).
     */
    public boolean isPivotAtTarget() {
        // This would require storing the target position - simplified for now
        return true; // TODO: implement proper target tracking
    }

    // === Roller Control Methods ===

    /**
     * Run the roller at intake speed (into robot).
     */
    public void runIntake() {
        rollerController.setReference(IntakeConstants.kIntakeRpm.get(), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    /**
     * Run the roller at outtake speed (out of robot).
     */
    public void runOuttake() {
        rollerController.setReference(IntakeConstants.kOuttakeRpm.get(), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    /**
     * Stop the roller motor.
     */
    public void stopRoller() {
        rollerMotor.stopMotor();
    }

    /**
     * Get the current roller velocity in RPM.
     */
    public AngularVelocity getRollerVelocityRpm() {
        return RPM.of(rollerEncoder.getVelocity());
    }

    // === Periodic ===

    @Override
    public void periodic() {
        // Update PID constants if they've changed
        updatePivotPID();
        updateRollerPID();

        // Telemetry
        SmartDashboard.putNumber("Intake/Pivot Degrees", getPivotDegrees());
        SmartDashboard.putNumber("Intake/Pivot Radians", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Intake/Roller RPM", getRollerVelocityRpm().in(RPM));
    }

    /**
     * Updates the pivot PID constants from NetworkTables if they've changed.
     */
    private void updatePivotPID() {
        int id = this.hashCode();
        if (IntakeConstants.kPPivot.hasChanged(id) || IntakeConstants.kIPivot.hasChanged(id) || 
            IntakeConstants.kDPivot.hasChanged(id) || IntakeConstants.kGPivot.hasChanged(id) || 
            IntakeConstants.kPivotToleranceRadians.hasChanged(id) ||
            IntakeConstants.kPivotMaxVelocityRps.hasChanged(id) || IntakeConstants.kPivotMaxAccelRps2.hasChanged(id)) {
            
            pivotConfig.closedLoop
                .p(IntakeConstants.kPPivot.get(), ClosedLoopSlot.kSlot0)
                .i(IntakeConstants.kIPivot.get(), ClosedLoopSlot.kSlot0)
                .d(IntakeConstants.kDPivot.get(), ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(IntakeConstants.kPivotToleranceRadians.get(), ClosedLoopSlot.kSlot0);
            
            if (IntakeConstants.kUseSmartMotion) {
                pivotConfig.closedLoop
                    .maxMotion
                        .maxVelocity(IntakeConstants.kPivotMaxVelocityRps.get(), ClosedLoopSlot.kSlot0)
                        .maxAcceleration(IntakeConstants.kPivotMaxAccelRps2.get(), ClosedLoopSlot.kSlot0);
            }
            
            pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            
            SmartDashboard.putString("Intake/Pivot/Status", 
                String.format("Updated: P=%.3f I=%.3f D=%.3f G=%.3f", 
                    IntakeConstants.kPPivot.get(), IntakeConstants.kIPivot.get(), 
                    IntakeConstants.kDPivot.get(), IntakeConstants.kGPivot.get()));
        }
    }

    /**
     * Updates the roller PID constants from NetworkTables if they've changed.
     */
    private void updateRollerPID() {
        int id = this.hashCode();
        if (IntakeConstants.kPRoller.hasChanged(id) || IntakeConstants.kIRoller.hasChanged(id) || 
            IntakeConstants.kDRoller.hasChanged(id) || IntakeConstants.kVRoller.hasChanged(id)) {
            
            rollerConfig.closedLoop
                .p(IntakeConstants.kPRoller.get(), ClosedLoopSlot.kSlot0)
                .i(IntakeConstants.kIRoller.get(), ClosedLoopSlot.kSlot0)
                .d(IntakeConstants.kDRoller.get(), ClosedLoopSlot.kSlot0)
                .velocityFF(IntakeConstants.kVRoller.get(), ClosedLoopSlot.kSlot0);
            
            rollerMotor.configure(rollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            
            SmartDashboard.putString("Intake/Roller/Status", 
                String.format("Updated: P=%.4f I=%.4f D=%.4f V=%.5f", 
                    IntakeConstants.kPRoller.get(), IntakeConstants.kIRoller.get(), 
                    IntakeConstants.kDRoller.get(), IntakeConstants.kVRoller.get()));
        }
    }
}
