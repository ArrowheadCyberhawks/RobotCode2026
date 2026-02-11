package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.intake.IntakeConstants.IntakePosition;

/**
 * Intake subsystem using REV Spark Flex motor controllers.
 * Uses RoboRIO-side ProfiledPIDController for smooth motion profiling.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex pivotMotor;
    private final SparkFlex rollerMotor;

    private final RelativeEncoder pivotEncoder;
    private final RelativeEncoder rollerEncoder;

    private final CANcoder pivotAbsoluteEncoder; // Optional: for absolute position feedback

    private SparkFlexConfig pivotConfig;
    private SparkFlexConfig rollerConfig;

    // Controllers
    private final ProfiledPIDController pivotController;
    private final SimpleMotorFeedforward pivotFeedforward;
    
    private double rollerTargetRPM = 0.0;
    /** Current high-level intake state (controls pivot + roller behavior) */
    private IntakeState intakeState = IntakeState.IDLE;

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

        // Create absolute encoder
        pivotAbsoluteEncoder = new CANcoder(IntakeConstants.kPivotAbsoluteEncoderId);

        // Create ProfiledPIDController with trapezoidal motion profile
        pivotController = new ProfiledPIDController(
            IntakeConstants.kPPivot.get(),
            IntakeConstants.kIPivot.get(),
            IntakeConstants.kDPivot.get(),
            new TrapezoidProfile.Constraints(
                IntakeConstants.kPivotMaxVelocityRps.get(),
                IntakeConstants.kPivotMaxAccelRps2.get()
            )
        );
        pivotController.setTolerance(IntakeConstants.kPivotToleranceRadians.get());

        // Create feedforward for gravity and velocity compensation
        pivotFeedforward = new SimpleMotorFeedforward(
            0.0,  // kS - static friction (usually not needed for position control)
            IntakeConstants.kVPivot.get(),
            IntakeConstants.kAPivot.get()
        );

        // Configure motors
        configurePivot();
        configureRoller();

        // Zero pivot at known position (stowed = 0 degrees)
        syncEncoders();

        SmartDashboard.putBoolean("Intake/UseProfiledPID", true);
    }

    /** High-level intake states that control pivot and roller behavior */
    public enum IntakeState {
        STOW, // pivot up and rollers stopped
        IDLE, // pivot down (ready) but rollers not running
        RUN   // pivot down and rollers running to intake
    }

    /** Request the intake to move into a high-level state. */
    public void setIntakeState(IntakeState state) {
        this.intakeState = state;
    }

    /** Read the current intake state. */
    public IntakeState getIntakeState() {
        return this.intakeState;
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

        // Apply configuration (roller uses simple voltage control)
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
     * Set the pivot to a specific angle using ProfiledPIDController.
     */
    public void setPivotTarget(Angle targetAngle) {
        double targetRadians = targetAngle.in(Radians);
        pivotController.setGoal(targetRadians);
    }

    /**
     * Stop the pivot motor and reset the controller.
     */
    public void stopPivot() {
        pivotMotor.stopMotor();
        pivotController.reset(getPivotAngle_Relative().in(Radians));
    }

    /**
     * Get the current pivot angle from the motor's relative encoder.
     */
    public Angle getPivotAngle_Relative() {
        return Radians.of(pivotEncoder.getPosition());
    }

    /**
     * Get the current pivot angle from the absolute encoder.
     */
    public Angle getPivotAngle_Absolute() {
        return Rotations.of(pivotAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * Reset the pivot encoder to a specific angle in degrees.
     */
    public void resetPivotEncoderToAngle(Angle angle) {
        double radians = angle.in(Radians);
        pivotEncoder.setPosition(radians);
        pivotController.reset(radians);
    }

    public void syncEncoders() {
        double absoluteRadians = getPivotAngle_Absolute().in(Radians);
        pivotEncoder.setPosition(absoluteRadians);
        pivotController.reset(absoluteRadians);
    }

    /**
     * Check if the pivot is at the target position (within tolerance).
     */
    public boolean isPivotAtTarget() {
        return pivotController.atGoal();
    }

    // === Roller Control Methods ===

    /**
     * Run the roller at intake speed (into robot).
     */
    public void runIntake() {
        rollerTargetRPM = IntakeConstants.kIntakeRpm.get();
    }

    /**
     * Run the roller at outtake speed (out of robot).
     */
    public void runOuttake() {
        rollerTargetRPM = IntakeConstants.kOuttakeRpm.get();
    }

    /**
     * Stop the roller motor.
     */
    public void stopRoller() {
        rollerTargetRPM = 0.0;
        rollerMotor.stopMotor();
    }

    /**
     * Get the current roller velocity in RPM.
     */
    public AngularVelocity getRollerVelocity() {
        return RPM.of(rollerEncoder.getVelocity());
    }

    // === Periodic ===

    @Override
    public void periodic() {
        switch (intakeState) {
            case STOW:
                // Pivot up and rollers stopped
                moveToPosition(IntakePosition.STOWED);
                stopRoller();
                break;
            case IDLE:
                // Pivot down (ready) but rollers not running
                moveToPosition(IntakePosition.INTAKE);
                stopRoller();
                break;
            case RUN:
                // Pivot down and rollers running
                moveToPosition(IntakePosition.INTAKE);
                runIntake();
                break;
            default:
                stopRoller();
                break;
        }

        SmartDashboard.putString("Intake/State", intakeState.name());

        // Run the ProfiledPIDController and apply output to motor
        updatePivotControl();
        
        // Update roller control
        updateRollerControl();
        
        // Update PID constants if they've changed
        updatePivotPID();

        // Telemetry
        SmartDashboard.putNumber("Intake/Pivot Relative Degrees", getPivotAngle_Relative().in(Degrees));
        SmartDashboard.putNumber("Intake/Pivot Absolute Degrees", getPivotAngle_Absolute().in(Degrees));
        SmartDashboard.putNumber("Intake/Pivot Target Degrees", Math.toDegrees(pivotController.getGoal().position));
        SmartDashboard.putNumber("Intake/Pivot Setpoint Degrees", Math.toDegrees(pivotController.getSetpoint().position));
        SmartDashboard.putNumber("Intake/Pivot Error Degrees", Math.toDegrees(pivotController.getPositionError()));
        SmartDashboard.putBoolean("Intake/Pivot At Goal", pivotController.atGoal());
        SmartDashboard.putNumber("Intake/Roller RPM", getRollerVelocity().in(RPM));
        SmartDashboard.putNumber("Intake/Roller Target RPM", rollerTargetRPM);
    }

    /**
     * Runs the ProfiledPIDController and applies the output to the pivot motor.
     */
    private void updatePivotControl() {
        // Get current position
        double currentPosition = getPivotAngle_Absolute().in(Radians);
        
        // Calculate PID output
        double pidOutput = pivotController.calculate(currentPosition);
        
        // Calculate feedforward (gravity compensation based on position)
        // The feedforward should include a gravity term that varies with angle
        double setpointVelocity = pivotController.getSetpoint().velocity;
        double feedforward = pivotFeedforward.calculate(setpointVelocity);
        
        // Add gravity compensation (kG * cos(angle))
        double gravityCompensation = IntakeConstants.kGPivot.get() * Math.cos(currentPosition);
        
        // Combine PID and feedforward
        double totalOutput = pidOutput + feedforward + gravityCompensation;
        
        // Clamp output to [-12, 12] volts
        totalOutput = Math.max(-12.0, Math.min(12.0, totalOutput));
        
        // Apply to motor
        pivotMotor.setVoltage(totalOutput);
        
        SmartDashboard.putNumber("Intake/Pivot/PID Output", pidOutput);
        SmartDashboard.putNumber("Intake/Pivot/FF Output", feedforward);
        SmartDashboard.putNumber("Intake/Pivot/Gravity Output", gravityCompensation);
        SmartDashboard.putNumber("Intake/Pivot/Total Output", totalOutput);
    }

    /**
     * Updates the roller motor control with simple voltage-based speed control.
     */
    private void updateRollerControl() {
        // Simple proportional control for roller
        // For better control, you could add a PID controller here too
        if (Math.abs(rollerTargetRPM) > 10.0) {
            // Very simple: target RPM / max RPM * 12V
            // This is a crude approach; for better control, tune kV
            double maxRPM = 5000.0; // Adjust based on your motor's free speed
            double voltage = (rollerTargetRPM / maxRPM) * 12.0;
            voltage = MathUtil.clamp(voltage, -12.0, 12.0);
            rollerMotor.setVoltage(voltage);
        } else {
            rollerMotor.stopMotor();
        }
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
            
            // Update ProfiledPIDController parameters
            pivotController.setP(IntakeConstants.kPPivot.get());
            pivotController.setI(IntakeConstants.kIPivot.get());
            pivotController.setD(IntakeConstants.kDPivot.get());
            pivotController.setConstraints(
            new TrapezoidProfile.Constraints(
                IntakeConstants.kPivotMaxVelocityRps.get(),
                IntakeConstants.kPivotMaxAccelRps2.get()
            )
            );
            pivotController.setTolerance(IntakeConstants.kPivotToleranceRadians.get());
            
            long timestamp = System.currentTimeMillis();
            SmartDashboard.putString("Intake/Pivot/Status", 
            String.format("Updated [%d]: P=%.3f I=%.3f D=%.3f G=%.3f MaxV=%.2f MaxA=%.2f", 
                timestamp,
                IntakeConstants.kPPivot.get(), IntakeConstants.kIPivot.get(), 
                IntakeConstants.kDPivot.get(), IntakeConstants.kGPivot.get(),
                IntakeConstants.kPivotMaxVelocityRps.get(), IntakeConstants.kPivotMaxAccelRps2.get()));
        }
    }
}
