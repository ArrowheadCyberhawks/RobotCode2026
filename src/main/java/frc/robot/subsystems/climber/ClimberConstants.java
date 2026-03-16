package frc.robot.subsystems.climber;
import frc.robot.util.LoggedTunableNumber;

public class ClimberConstants {
        // CAN ID for the motor
        public static final int kClimberMotorId = 30; 
    
        // Gear ratio: motor rotations per climber rotation
        public static final double kClimberGearRatio = 45;
    
        // Defines PID
        public static final LoggedTunableNumber kPClimb = new LoggedTunableNumber("Climber/P", 0.0);
        public static final LoggedTunableNumber kIClimb = new LoggedTunableNumber("Climber/I", 0.0);
        public static final LoggedTunableNumber kDClimb = new LoggedTunableNumber("Climber/D", 0.0);
        public static final LoggedTunableNumber kGClimb = new LoggedTunableNumber("Climber/G", 0.0);
        public static final LoggedTunableNumber kVClimb = new LoggedTunableNumber("Climber/V", 0.0);
        public static final LoggedTunableNumber kAClimb = new LoggedTunableNumber("Climber/A", 0.0);

        public static final LoggedTunableNumber climberDownSpeed = new LoggedTunableNumber("climberDownSpeed", -0.1);
        public static final LoggedTunableNumber climberUpSpeed = new LoggedTunableNumber("climberUpSpeed", 0.1);

        // TODO: get these values and then set to doubles
        public static final LoggedTunableNumber kClimberMaxPosition = new LoggedTunableNumber("Climber/MaxPosition", 10.0);
        public static final LoggedTunableNumber kClimberMinPosition = new LoggedTunableNumber("Climber/MinPosition", 0.0); 
    // test befefore adjusting values
    // climberUpSpeed pulls the robot up the tower and is a positive value
        private ClimberConstants() {}

}