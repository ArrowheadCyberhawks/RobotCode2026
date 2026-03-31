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
    private LEDConstants() {
    }

    public static final int kCANdleId = 56;
    public static final CANBus kCANBus = CANBus.roboRIO();

    //public static final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
    public static final RGBW Color kGreen = RGBWColor.fromHex("#24f00000").orElseThrow();
    //public static final RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(0.5); //not needed
    //public static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
    public static final RGBW Color kPurple = RGBWColor.fromHex("#D924f000").orElseThrow();
    public static final RGBWColor kRed = RGBWColor.fromHex("#D9000000").orElseThrow(); 
    public static final RGBWColor kYellow = RGBWColor.fromHex("#D9D90000").orElseThrow();
    public static final RGBWColor kWhite = RGBWColor.fromHex("#D9FFFFFF").orElseThrow();

    /* assignments will be: intaking (purple), intaking while shooting (green), 
    *  herding/reverse intake (yellow), defense/intake up (blue), warning (red flashing)
    */
   public enum intakestate {
    Intaking();
    IntakingShooting ();
    ReverseIntaking ();

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
