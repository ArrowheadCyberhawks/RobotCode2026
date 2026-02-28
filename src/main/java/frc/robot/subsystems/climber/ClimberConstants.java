package frc.robot.subsystems.climber;

public class ClimberConstants {
        // CAN IDs
        public static final int kLeftMotorId = 30;
        public static final int kRightMotorId = 31;
    
        // Gear ratio: motor rotations per climber rotation
        public static final double kClimberGearRatio = 100.0;
    
        // Motion Magic constraints (motor-side, rotations)
        public static final double kClimbCruiseRps = 2.0;     // motor rotations/sec
        public static final double kClimbAccelRps2 = 4.0;    // motor rotations/sec^2
    
        // PID + FF
        public static final double kPClimb = 60.0;
        public static final double kIClimb = 0.0;
        public static final double kDClimb = 4.0;
        public static final double kGClimb = 0.4;  // gravity feedforward
        public static final double kVClimb = 0.0;
        public static final double kAClimb = 0.0;
    
        private ClimberConstants() {}

}
