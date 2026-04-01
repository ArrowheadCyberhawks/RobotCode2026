package frc.robot.subsystems.led;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.*;

/**
 * Constants for the CANdle LED subsystem.
 *
 * These are extracted from the standalone CANdle example Robot class
 * so they can be reused from {@link LEDSubsystem}.
 */
public final class LEDConstants {
    public static final int kCANdleId = 56;
    public static final CANBus kCANBus = CANBus.roboRIO();

    public static final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
    public static final RGBWColor kBlue = RGBWColor.fromHex("rgba(0, 42, 255, 0)").orElseThrow(); 
    public static final RGBWColor kRed = RGBWColor.fromHex("rgba(255, 0, 0, 0)").orElseThrow(); 
    public static final RGBWColor kYellow = RGBWColor.fromHex("rgba(255, 242, 0, 0)").orElseThrow();
    public static final RGBWColor kWhite = RGBWColor.fromHex("#D9FFFFFF").orElseThrow();
    public static final RGBWColor kPurple = RGBWColor.fromHex("#D924f000").orElseThrow();
    public static final RGBWColor kPink = RGBWColor.fromHex("#ec108b").orElseThrow();

    /* assignments will be: intaking (green), intaking while shooting (purple), default (pink),
    *  herding/reverse intake (yellow), defense/intake up (blue), warning (red flashing)
    */
    public enum LEDState {
        DEFAULT(kPink), //not doing anything
        HERD(kYellow), //reverse intake
        INTAKE(kGreen), //intaking
        INTAKESHOOT(kPurple), //intaking while shooting
        DEFENSE (kBlue), //intake up or defense mode
        WARNING (kRed), //warning
        SHOOTING (kWhite); //shooting without intaking

        public final RGBWColor color;

        LEDState(RGBWColor color) {
            this.color = color;
        }
    }
    
    /*
     * Start and end index for LED animations.
     * 0-7 are onboard, 8-399 are an external strip.
     * CANdle supports 8 animation slots (0-7).
     */
    public static final int kSlot0StartIdx = 8;
    public static final int kSlot0EndIdx = 37;

    public static final int kSlot1StartIdx = 38;
    public static final int kSlot1EndIdx = 67;

    }
    
