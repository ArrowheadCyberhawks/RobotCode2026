package frc.robot.subsystems.hopper;

import frc.robot.util.LoggedTunableNumber;

public final class HopperConstants {
    public final class hopperMotor {
        public static final int hopperMotorId = 12; 
        public static final int kickerMotorId = 11;
    } 

        public static final LoggedTunableNumber kPHopper = 
            new LoggedTunableNumber("Hopper/PIDConstants/kPHopper", 0.0);
        public static final LoggedTunableNumber kIHopper = 
            new LoggedTunableNumber("Hopper/PIDConstants/kIHopper", 0.0);
        public static final LoggedTunableNumber kDHopper = 
            new LoggedTunableNumber("Hopper/PIDConstants/kDHopper", 0.0);
        public static final LoggedTunableNumber kVHopper = 
            new LoggedTunableNumber("Hopper/PIDConstants/kVHopper", 0.12);
        public static final LoggedTunableNumber kSHopper = 
            new LoggedTunableNumber("Hopper/PIDConstants/kSHopper", 0.0);
        public static final LoggedTunableNumber kHopperMaxVelocityRps = 
            new LoggedTunableNumber("Hopper/PIDConstants/kHopperMaxVelocityRps", 5.0); // Example value
        public static final LoggedTunableNumber kHopperMaxAccelRps2 = 
            new LoggedTunableNumber("Hopper/PIDConstants/kHopperMaxAccelRps2", 10.0); // Example value

        public static final LoggedTunableNumber kPKicker = 
            new LoggedTunableNumber("Hopper/PIDConstants/kPKicker", 0.0003);
        public static final LoggedTunableNumber kIKicker = 
            new LoggedTunableNumber("Hopper/PIDConstants/kIKicker", 0.0);
        public static final LoggedTunableNumber kDKicker = 
            new LoggedTunableNumber("Hopper/PIDConstants/kDKicker", 0.0);
        public static final LoggedTunableNumber kVKicker = 
            new LoggedTunableNumber("Hopper/PIDConstants/kVKicker", 0.0019);
        public static final LoggedTunableNumber kSKicker = 
            new LoggedTunableNumber("Hopper/PIDConstants/kSKicker", 0.0);

        public static final LoggedTunableNumber kHopperRpm = 
            new LoggedTunableNumber("Hopper/HopperRpm", 5000.0);
        public static final LoggedTunableNumber kKickerRpm = 
            new LoggedTunableNumber("Hopper/KickerRpm", 3000.0); 
        public static final LoggedTunableNumber KickerReverseRpm = 
            new LoggedTunableNumber("Hopper/KickerReverse", -2000.0);
        public static final LoggedTunableNumber kHopperReversedRpm = 
            new LoggedTunableNumber("Hopper/HopperReversedRpm", -2000.0);
        private HopperConstants() {}
    }
    