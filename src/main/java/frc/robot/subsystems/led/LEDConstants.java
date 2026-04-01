package frc.robot.subsystems.led;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.*;

/**
 * Constants for the CANdle LED subsystem.
 *
 * These are extracted from the standalone CANdle example Robot class
 * so they can be reused from {@link LEDSubsystem}.
 */
public final class LEDConstants {
    private LEDConstants() {
    }

    public static final int kCANdleId = 56;
    public static final CANBus kCANBus = CANBus.roboRIO();

    public static final RGBWColor kGreen = RGBWColor.fromHex("rgba(38, 255, 0, 1)").orElseThrow();
    public static final RGBWColor kPurple = RGBWColor.fromHex("rgba(153, 0, 255, 1)").orElseThrow();
    public static final RGBWColor kRed = RGBWColor.fromHex("rgba(255, 0, 0, 1)").orElseThrow();
    public static final RGBWColor kBlue = RGBWColor.fromHex("rgba(0, 102, 255, 1)").orElseThrow();
    public static final RGBWColor kYellow = RGBWColor.fromHex("rgba(217, 217, 0, 1)").orElseThrow();
    public static final RGBWColor kWhite = RGBWColor.fromHex("rgba(217, 217, 217, 1)").orElseThrow();
    public static final RGBWColor kPink = RGBWColor.fromHex("rgba(236, 16, 139, 1)").orElseThrow();


    /* assignments will be: intaking (purple), intaking while shooting (green), 
    *  herding/reverse intake (yellow), defense/intake up (blue), warning (red flashing)
    */
    public enum LEDState {
        DEFAULT(kPink),
        DEFENSE(kBlue),
        TRENCH(kWhite),
        SHOOTINTAKE(kPink),
        INTAKE(kGreen),
        HERD(kYellow),
        WARNING(kRed);

        public final RGBWColor color;

        LEDState(RGBWColor color) {
            this.color = color;
        }

        public RGBWColor getColor() {
            return color;
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
