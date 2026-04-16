package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;

/**
 * Intake subsystem using REV Spark Flex motor controllers.
 * Uses WPILib ProfiledPIDController (RobotRIO-side) for smooth motion profiling.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex pivotMotor;
    private final TalonFX rollerMotor;

    private final RelativeEncoder pivotEncoder;

    private final CANcoder pivotAbsoluteEncoder;

    private SparkFlexConfig pivotConfig;

    // Controllers - WPILib ProfiledPIDController only
    private final ProfiledPIDController pivotController;
    private final ArmFeedforward pivotFeedforward;
    
    private double rollerTargetPercent = 0.0;

    // Timing for jam detection / unjam behavior
    private double jamOverCurrentStartTime = Double.NaN;
    private double unjamStartTime = Double.NaN;
    /** Current high-level intake state (controls pivot + roller behavior) */
    private IntakeState intakeState = IntakeState.STOW;

    public IntakeSubsystem() {
        // Create motors
        pivotMotor = new SparkFlex(IntakeConstants.kPivotMotorId, MotorType.kBrushless);
        rollerMotor = new TalonFX(IntakeConstants.kRollerMotorId);

        pivotEncoder = pivotMotor.getEncoder();

        // Create configs
        pivotConfig = new SparkFlexConfig();

        // Create absolute encoder
        pivotAbsoluteEncoder = new CANcoder(IntakeConstants.kIntakePivotEncoderId);
        //TODO: FIX DISCONTINUITY POINT OF ABSOLUTE ENCODER

        // Create WPILib ProfiledPIDController with trapezoidal motion profile (RobotRIO-side)
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
        pivotFeedforward = new ArmFeedforward(
            IntakeConstants.kSPivot.get(),
            IntakeConstants.kGPivot.get(),
            IntakeConstants.kVPivot.get(),
            IntakeConstants.kAPivot.get()
        );

        // Configure motors (no PID on motor controllers - using WPILib only)
        configurePivot();
        configureRoller();

        // Zero pivot at known position (stowed = 0 degrees)
        syncEncoders();
    }

    private void configurePivot() {
        // Encoder conversion: motor rotations → radians of pivot
        pivotConfig.encoder
            .positionConversionFactor(IntakeConstants.kPivotMotorGearRatio * 2.0 * Math.PI) // motor rotations to pivot radians
            .velocityConversionFactor(IntakeConstants.kPivotMotorGearRatio * 2.0 * Math.PI / 60.0); // motor RPM to pivot radians per second

        // Motor output settings - NO PID configured here (using WPILib ProfiledPIDController)
        pivotConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false);

        // Soft limits (converted to radians)
        pivotConfig.softLimit
            .forwardSoftLimit(Math.toRadians(IntakeConstants.kPivotMaxDegrees))
            .reverseSoftLimit(Math.toRadians(IntakeConstants.kPivotMinDegrees))
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(false);

        pivotConfig.smartCurrentLimit(40, 30);

        // Apply configuration
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void configureRoller() {
        // Configure TalonFX for simple percent output control
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 60.0;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = 55.0;
        rollerMotor.getConfigurator().apply(cfg);
    }

    /** Request the intake to move into a high-level state. */
    public void setIntakeState(IntakeState state) {
        this.intakeState = state;
    }

    /** Read the current intake state. */
    public IntakeState getIntakeState() {
        return this.intakeState;
    }

    /** Toggle the roller motors on or off when the intake is extended. Has no effect if the intake is stowed. */
    public void toggleRunState() {
        if (intakeState == IntakeState.RUN) {
            setIntakeState(IntakeState.IDLE);
        } else if (intakeState == IntakeState.IDLE) {
            setIntakeState(IntakeState.RUN);
        }
    }

    /** Toggle the intake between stowed and ready (extended) states. If the intake is currently running, it will be stopped and stowed. */
    public void toggleExtendState() {
        if (intakeState == IntakeState.STOW) {
            setIntakeState(IntakeState.IDLE);
        } else {
            setIntakeState(IntakeState.STOW);
        }
    }

    /**
     * Set the pivot to a specific angle using WPILib ProfiledPIDController.
     */
    public void setPivotTarget(Angle targetAngle) {
        pivotController.setGoal(targetAngle.in(Radians));
    }

    /**
     * Stop the pivot motor and reset the controller.
     */
    public void stopPivot() {
        pivotMotor.stopMotor();
        pivotController.reset(getPivotAngleAbsolute().in(Radians));
    }

    /**
     * Get the current pivot angle from the absolute encoder.
     */
    public Angle getPivotAngleAbsolute() {
        return Rotations.of(pivotAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * IntakeConstants.kPivotEncoderGearRatio);
    }

    public void resetPivotEncoder() {
        pivotAbsoluteEncoder.setPosition(0.0);
        pivotEncoder.setPosition(0.0);
        pivotController.reset(0.0);
    }

    /**
     * Sync the controller to match the current absolute encoder position.
     * This is useful on startup or after a reset.
     */
    public void syncEncoders() {
        double absoluteRadians = getPivotAngleAbsolute().in(Radians);
        pivotEncoder.setPosition(absoluteRadians); // Sync relative encoder to absolute position
        pivotController.reset(absoluteRadians);
    }

    /**
     * Check if the pivot is at the target position (within tolerance).
     */
    public boolean isPivotAtTarget() {
        return pivotController.atGoal();
    }

    /**
     * Stop the roller motor.
     */
    public void stopRoller() {
        rollerTargetPercent = 0.0;
        rollerMotor.set(0.0);
        jamOverCurrentStartTime = Double.NaN;
        unjamStartTime = Double.NaN;
    }

    /**
     * Get the current roller motor velocity in RPM.
     */
    public AngularVelocity getRollerVelocity() {
        // TalonFX velocity is in rotations per second by default
        return rollerMotor.getVelocity().getValue();
    }


    @Override
    public void periodic() {
        if (intakeState == IntakeState.MANUAL)
            return; // skip everything in manual mode - manual control methods will handle motor outputs directly

        setPivotTarget(intakeState.getPivotTarget());
        rollerTargetPercent = intakeState.getRollerTarget();

        Logger.recordOutput("Intake/State", intakeState.toString());

        //checkRollerJam();
        updatePivotControl();
        updateRollerControl();
        updatePivotPID();

        // Telemetry
        Logger.recordOutput("Intake/Pivot Absolute Degrees", getPivotAngleAbsolute().in(Degrees));
        Logger.recordOutput("Intake/Pivot Relative Degrees", pivotEncoder.getPosition());
        Logger.recordOutput("Intake/Pivot Target Degrees", Math.toDegrees(pivotController.getGoal().position));
        Logger.recordOutput("Intake/Pivot Setpoint Degrees", Math.toDegrees(pivotController.getSetpoint().position));
        Logger.recordOutput("Intake/Pivot Error Degrees", Math.toDegrees(pivotController.getPositionError()));
        Logger.recordOutput("Intake/Roller RPM", getRollerVelocity().in(RPM));
        Logger.recordOutput("Intake/Roller Target Percent", rollerTargetPercent);
    }

    /**
     * Runs the WPILib ProfiledPIDController and applies the output to the pivot motor.
     */
    private void updatePivotControl() {
        // Get current position
        double currentPosition = getPivotAngleAbsolute().in(Radians);
        
        // Calculate PID output
        double pidOutput = pivotController.calculate(currentPosition);
        
        // Calculate feedforward (gravity compensation based on position)
        double setpointVelocity = pivotController.getSetpoint().velocity;
        double feedforward = pivotFeedforward.calculate(pivotController.getSetpoint().position, setpointVelocity);
        
        // Add gravity compensation (kG * cos(angle))
        double gravityCompensation = IntakeConstants.kGPivot.get() * Math.cos(currentPosition);
        
        // Combine PID and feedforward
        double totalOutput = pidOutput + feedforward + gravityCompensation;
        
        // Clamp output to [-12, 12] volts
        totalOutput = Math.max(-12.0, Math.min(12.0, totalOutput));
        
        // Apply to motor
        pivotMotor.setVoltage(totalOutput);
        
        Logger.recordOutput("Intake/Pivot/PID Output", pidOutput);
        Logger.recordOutput("Intake/Pivot/FF Output", feedforward);
        Logger.recordOutput("Intake/Pivot/Gravity Output", gravityCompensation);
        Logger.recordOutput("Intake/Pivot/Total Output", totalOutput);
        Logger.recordOutput("Intake/Pivot Current", pivotMotor.getOutputCurrent());
        Logger.recordOutput("Intake/Roller Current", rollerMotor.getSupplyCurrent().getValueAsDouble());
    }

    /**
     * Detects roller jams based on current draw and runs an automatic unjam cycle.
     * If the roller current stays above the configured threshold for longer than
     * the configured duration while running forward in RUN state, the roller
     * reverses briefly to clear the jam and then returns to normal operation.
     */
    private void checkRollerJam() {
        double now = Timer.getFPGATimestamp();
        double rollerCurrent = rollerMotor.getSupplyCurrent().getValueAsDouble();
        boolean rollerForwardDemand = intakeState == IntakeState.RUN && rollerTargetPercent > 0.1;

        if (intakeState == IntakeState.RUN) {
            if (rollerForwardDemand) {
                if (rollerCurrent > IntakeConstants.kRollerJamCurrentThreshold.get()) {
                    if (Double.isNaN(jamOverCurrentStartTime)) {
                        jamOverCurrentStartTime = now;
                    } else if (now - jamOverCurrentStartTime >=
                            IntakeConstants.kRollerJamDurationSeconds.get()) {
                        // Start unjamming by switching to UNJAM state
                        intakeState = IntakeState.UNJAM;
                        rollerTargetPercent = IntakeState.UNJAM.getRollerTarget();
                        unjamStartTime = now;
                    }
                } else {
                    jamOverCurrentStartTime = Double.NaN;
                }
            } else {
                jamOverCurrentStartTime = Double.NaN;
            }
        } else if (intakeState == IntakeState.UNJAM) {
            // During UNJAM, keep timers and switch back to RUN after unjam time
            if (Double.isNaN(unjamStartTime)) {
                unjamStartTime = now;
            }

            if (now - unjamStartTime >= IntakeConstants.kRollerUnjamDurationSeconds.get()) {
                intakeState = IntakeState.RUN;
                rollerTargetPercent = IntakeState.RUN.getRollerTarget();
                jamOverCurrentStartTime = Double.NaN;
                unjamStartTime = Double.NaN;
            }

        } else {
            // Other states: clear timers
            jamOverCurrentStartTime = Double.NaN;
            unjamStartTime = Double.NaN;
        }

        Logger.recordOutput("Intake/Roller/Current", rollerCurrent);
    }

    /**
     * Updates the roller motor control with simple voltage-based speed control.
     */
    private void updateRollerControl() {
        // Simple proportional control for roller
        if (Math.abs(rollerTargetPercent) > 0.0) {
            rollerMotor.set(rollerTargetPercent);
        } else {
            rollerMotor.set(0.0);
        }
    }

    /**
     * Updates the pivot PID constants from NetworkTables if they've changed.
     */
    private void updatePivotPID() {
        int id = this.hashCode();
        if (IntakeConstants.kPPivot.hasChanged(id) || IntakeConstants.kIPivot.hasChanged(id) || 
            IntakeConstants.kDPivot.hasChanged(id) || IntakeConstants.kGPivot.hasChanged(id) || 
            IntakeConstants.kVPivot.hasChanged(id) || IntakeConstants.kAPivot.hasChanged(id) ||
            IntakeConstants.kSPivot.hasChanged(id) ||
            IntakeConstants.kPivotToleranceRadians.hasChanged(id) ||
            IntakeConstants.kPivotMaxVelocityRps.hasChanged(id) || IntakeConstants.kPivotMaxAccelRps2.hasChanged(id)) {
            
            // Update WPILib ProfiledPIDController parameters
            pivotController.setP(IntakeConstants.kPPivot.get());
            pivotController.setI(IntakeConstants.kIPivot.get());
            pivotController.setD(IntakeConstants.kDPivot.get());
            pivotController.setConstraints(
                new TrapezoidProfile.Constraints(
                    IntakeConstants.kPivotMaxVelocityRps.get(),
                    IntakeConstants.kPivotMaxAccelRps2.get()
                )
            );

            pivotFeedforward.setKg(IntakeConstants.kGPivot.get());
            pivotFeedforward.setKv(IntakeConstants.kVPivot.get());
            pivotFeedforward.setKa(IntakeConstants.kAPivot.get());
            pivotFeedforward.setKs(IntakeConstants.kSPivot.get());
            pivotController.setTolerance(IntakeConstants.kPivotToleranceRadians.get());
        }
    }

    public Command manualPivotCommand(DoubleSupplier pivotPercent) {
        IntakeState previousState = intakeState;
        this.intakeState = IntakeState.MANUAL; // Set to manual mode to bypass automatic control in periodic()
        return this.runEnd(() -> {
            double pivotSpeed = pivotPercent.getAsDouble();
            pivotMotor.set(pivotSpeed);
        }, () -> {
            pivotMotor.stopMotor();
            this.intakeState = previousState; // Restore previous state when command ends
        });
    }
}
