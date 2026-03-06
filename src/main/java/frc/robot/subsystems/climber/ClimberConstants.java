package frc.robot.subsystems.climber;
import frc.robot.util.LoggedTunableNumber;

public class ClimberConstants {
        // CAN IDs
        public static final int kClimberId = 30;
        public static final int kClimberMotorId = 31;
    
        // Gear ratio: motor rotations per climber rotation
        public static final double kClimberGearRatio = 45;
    
        // PID
        public static final LoggedTunableNumber kPClimb = new LoggedTunableNumber("Climber/P", 0.0);
        public static final LoggedTunableNumber kIClimb = new LoggedTunableNumber("Climber/I", 0.0);
        public static final LoggedTunableNumber kDClimb = new LoggedTunableNumber("Climber/D", 0.0);
        public static final LoggedTunableNumber kGClimb = new LoggedTunableNumber("Climber/G", 0.0);
        public static final LoggedTunableNumber kVClimb = new LoggedTunableNumber("Climber/V", 0.0);
        public static final LoggedTunableNumber kAClimb = new LoggedTunableNumber("Climber/A", 0.0);
    
        private ClimberConstants() {}

}
