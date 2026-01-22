package frc.robot.subsystems.intake;

public final class IntakeConstants {

    // CAN IDs
    public static final int kPivotMotorId = 10;
    public static final int kRollerMotorId = 11;

    // Gear ratio: motor rotations per pivot rotation
    // Example: 100:1 reduction → 100 motor revs = 1 pivot rev
    public static final double kPivotGearRatio = 100.0;

    // Motion Magic constraints (motor-side, rotations)
    public static final double kPivotCruiseRps = 2.0;     // motor rotations/sec
    public static final double kPivotAccelRps2 = 4.0;    // motor rotations/sec^2

    // PID + FF
    public static final double kPPivot = 60.0;
    public static final double kIPivot = 0.0;
    public static final double kDPivot = 4.0;
    public static final double kGPivot = 0.4;  // gravity feedforward
    public static final double kVPivot = 0.0;
    public static final double kAPivot = 0.0;

    // Roller PID (velocity control)
    public static final double kPRoller = 0.12;
    public static final double kIRoller = 0.0;
    public static final double kDRoller = 0.001;
    public static final double kVRoller = 0.12;

    // Roller speeds (RPS)
    public static final double kIntakeRps = 40.0;
    public static final double kOuttakeRps = -40.0;


    public enum IntakePosition {
        STOWED(0.0),
        INTAKE(55.0);

        public final double degrees;

        IntakePosition(double degrees) {
            this.degrees = degrees;
        }
    }

    private IntakeConstants() {}
}
