package frc.robot.subsystems.led;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
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

    public static final RGBWColor kGreen = RGBWColor.fromHex("#26ff00ff").orElseThrow();
    public static final RGBWColor kPurple = RGBWColor.fromHex("#9900ffff").orElseThrow();
    public static final RGBWColor kRed = RGBWColor.fromHex("#ff0000ff").orElseThrow();
    public static final RGBWColor kBlue = RGBWColor.fromHex("#0066ffff").orElseThrow();
    public static final RGBWColor kYellow = RGBWColor.fromHex("#d9d900ff").orElseThrow();
    public static final RGBWColor kWhite = RGBWColor.fromHex("#d9d9d9ff").orElseThrow();
    public static final RGBWColor kPink = RGBWColor.fromHex("#ec108bff").orElseThrow();

    private enum AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
    }

    /* assignments will be: intaking (purple), intaking while shooting (green), 
    *  herding/reverse intake (yellow), defense/intake up (blue), warning (red flashing)
    */
    public enum LEDState {
        DEFAULT(new SingleFadeAnimation(0, kSlot0EndIdx).withColor(kPink).withFrameRate(1)),
        DISABLED(new SolidColor(0, kSlot0EndIdx).withColor(kPink)),
        DEFENSE(new SolidColor(0, kSlot0EndIdx).withColor(kBlue)),
        TRENCH(new SolidColor(0, kSlot0EndIdx).withColor(kWhite)),
        SHOOTINTAKE(new SolidColor(0, kSlot0EndIdx).withColor(kPink)),
        INTAKE(new SolidColor(0, kSlot0EndIdx).withColor(kGreen)),
        HERD(new SolidColor(0, kSlot0EndIdx).withColor(kYellow)),
        WARNING(new StrobeAnimation(0, kSlot0EndIdx).withColor(kRed).withFrameRate(1));

        public final ControlRequest request;

        LEDState(ControlRequest request) {
            this.request = request;
        }

        public ControlRequest getRequest() {
            return request;
        }
    }

    /* 
     * Start and end index for LED animations.
     * 0-7 are onboard, 8-399 are an external strip.
     * CANdle supports 8 animation slots (0-7).
     */
    public static final int kSlot0StartIdx = 0;
    public static final int kSlot0EndIdx = 67;

    public static final int kSlot1StartIdx = 61;
    public static final int kSlot1EndIdx = 67;
}
