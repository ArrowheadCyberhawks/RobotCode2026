package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.util.LoggedTunableNumber;

public final class IntakeConstants {

    // CAN IDs
    public static final int kPivotMotorId = 14;
    public static final int kRollerMotorId = 15;
    public static final int kIntakePivotEncoderId = 55;

    // Gear ratio: motor rotations per pivot rotation
    // Example: 100:1 reduction → 100 motor revs = 1 pivot rev
    public static final double kPivotEncoderGearRatio = 24.0/42.0;
    public static final double kPivotMotorGearRatio = 1/9.0 * 1/9.0 * kPivotEncoderGearRatio; // 9:1 reduction from motor to pivot, then encoder ratio
    public static final double kRollerGearRatio = 1/5.0;

    // Smart Motion constraints (rotations/sec)
    public static final LoggedTunableNumber kPivotMaxVelocityRps = 
        new LoggedTunableNumber("Intake/Pivot/MaxVelocityRps", 4.0);
    public static final LoggedTunableNumber kPivotMaxAccelRps2 = 
        new LoggedTunableNumber("Intake/Pivot/MaxAccelRps2", 8.0);

    // PID + FF for pivot (position control)
    public static final LoggedTunableNumber kPPivot = 
        new LoggedTunableNumber("Intake/Pivot/kP", 4.0);
    public static final LoggedTunableNumber kIPivot = 
        new LoggedTunableNumber("Intake/Pivot/kI", 0.0);
    public static final LoggedTunableNumber kDPivot = 
        new LoggedTunableNumber("Intake/Pivot/kD", 0.0);
    public static final LoggedTunableNumber kGPivot = 
        new LoggedTunableNumber("Intake/Pivot/kG", 0.1);
    public static final LoggedTunableNumber kVPivot = 
        new LoggedTunableNumber("Intake/Pivot/kV", 0.0);
    public static final LoggedTunableNumber kAPivot = 
        new LoggedTunableNumber("Intake/Pivot/kA", 0.0);
    public static final LoggedTunableNumber kSPivot = 
        new LoggedTunableNumber("Intake/Pivot/kS", 0.1);

    // Pivot position tolerance (radians)
    public static final LoggedTunableNumber kPivotToleranceRadians = 
        new LoggedTunableNumber("Intake/Pivot/ToleranceRadians", 0.02);

    // Pivot soft limits (degrees)
    public static final double kPivotMinDegrees = -8.0;
    public static final double kPivotMaxDegrees = 125.0;

    // Roller PID (velocity control)
    public static final LoggedTunableNumber kPRoller = 
        new LoggedTunableNumber("Intake/Roller/kP", 0.0001);
    public static final LoggedTunableNumber kIRoller = 
        new LoggedTunableNumber("Intake/Roller/kI", 0.0);
    public static final LoggedTunableNumber kDRoller = 
        new LoggedTunableNumber("Intake/Roller/kD", 0.0);
    public static final LoggedTunableNumber kVRoller = 
        new LoggedTunableNumber("Intake/Roller/kV", 0.00018);

    // Roller speeds (RPM)
    public static final LoggedTunableNumber kIntakeRpm = 
        new LoggedTunableNumber("Intake/Roller/IntakeRpm", 2400.0);

    // Roller jam detection and unjam behavior
    // If roller current stays above this threshold for longer than the
    // configured duration while running, the roller will briefly reverse
    // to clear the jam and then resume normal operation.
    public static final LoggedTunableNumber kRollerJamCurrentThreshold =
        new LoggedTunableNumber("Intake/Roller/JamCurrentThreshold", 50.0);
    public static final LoggedTunableNumber kRollerJamDurationSeconds =
        new LoggedTunableNumber("Intake/Roller/JamDurationSeconds", 2.0);
    public static final LoggedTunableNumber kRollerUnjamDurationSeconds =
        new LoggedTunableNumber("Intake/Roller/UnjamDurationSeconds", 0.5);

    /** High-level intake states that control pivot and roller behavior */
    public enum IntakeState {
        STOW(Degrees.of(125.0), 0.0), // pivot up and rollers stopped
        TRENCH(Degrees.of(50.0), 0.0), // pivot down (ready) but rollers not running
        IDLE(Degrees.of(0.0), 0.0), // pivot down (ready) but rollers not running
        RUN(Degrees.of(-3.0), 1.0),   // pivot down and rollers running to intake
        UNJAM(Degrees.of(-3.0), -1.0), // pivot down, rollers reverse briefly to clear jams
        REVERSE(Degrees.of(0.0), -1.00);

        public final Angle pivotTarget;
        public final double rollerTarget;

        IntakeState(Angle pivotAngle, double rollerVelocity) {
            this.pivotTarget = pivotAngle;
            this.rollerTarget = rollerVelocity;
        }

        /**
         * Get target pivot angle for this state.
         * @return target pivot angle, where 0 is pointing straight forward and positive is rotating upward
         */
        public Angle getPivotTarget() {
            return pivotTarget;
        }

        /**
         * Get target roller duty cycle as a percentage (0.0 to 1.0).
         */
        public double getRollerTarget() {
            return rollerTarget;
        }
    }

    private IntakeConstants() {}
}
