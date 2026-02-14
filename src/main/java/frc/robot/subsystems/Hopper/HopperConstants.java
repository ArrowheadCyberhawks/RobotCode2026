package frc.robot.subsystems.hopper;

import frc.robot.util.LoggedTunableNumber;

public final class HopperConstants {
    public final class hopperMotor {
        public static final int hopperMotor = 12; 
        public static final int kickerMotor = 13;
    } 

        public static final LoggedTunableNumber kPHopper = 
            new LoggedTunableNumber("Hopper/PIDConstants/kPHopper", 0.0);
        public static final LoggedTunableNumber kIHopper = 
            new LoggedTunableNumber("Hopper/PIDConstants/kIHopper", 0.0);
        public static final LoggedTunableNumber kDHopper = 
            new LoggedTunableNumber("Hopper/PIDConstants/kDHopper", 0.0);
        public static final LoggedTunableNumber kHopperMaxVelocityRps = 
            new LoggedTunableNumber("Hopper/PIDConstants/kHopperMaxVelocityRps", 5.0); // Example value
        public static final LoggedTunableNumber kHopperMaxAccelRps2 = 
            new LoggedTunableNumber("Hopper/PIDConstants/kHopperMaxAccelRps2", 10.0); // Example value

        public static final LoggedTunableNumber kPKicker = 
            new LoggedTunableNumber("Hopper/PIDConstants/kPKicker", 0.0);
        public static final LoggedTunableNumber kIKicker = 
            new LoggedTunableNumber("Hopper/PIDConstants/kIKicker", 0.0);
        public static final LoggedTunableNumber kDKicker = 
            new LoggedTunableNumber("Hopper/PIDConstants/kDKicker", 0.0);
        public static final LoggedTunableNumber kKickerMaxVelocityRps = 
            new LoggedTunableNumber("Hopper/PIDConstants/kKickerMaxVelocityRps", 5.0); // Example value
        public static final LoggedTunableNumber kKickerMaxAccelRps2 = 
            new LoggedTunableNumber("Hopper/PIDConstants/kKickerMaxAccelRps2", 10.0); // Example value

        public static final LoggedTunableNumber kHopperRpm = 
            new LoggedTunableNumber("Hopper/HopperRpm", 100.0); // Example value
        public static final LoggedTunableNumber kKickerRpm = 
            new LoggedTunableNumber("Hopper/KickerRpm", 100.0); // Example value
        public static final LoggedTunableNumber KickerReverseRpm = 
            new LoggedTunableNumber("Hopper/KickerReverse", -100.0); // Example value
        public static final LoggedTunableNumber kHopperReversedRpm = 
            new LoggedTunableNumber("Hopper/HopperReversedRpm", -100.0); // Example value
    }
    