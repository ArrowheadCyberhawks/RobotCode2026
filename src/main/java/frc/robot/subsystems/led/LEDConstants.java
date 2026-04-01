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

    public static final RGBWColor kGreen = RGBWColor.fromHex("rgba(38, 255, 0, 1)").orElseThrow();
    public static final RGBWColor kPurple = RGBWColor.fromHex("rgba(153, 0, 255, 1)").orElseThrow();
    public static final RGBWColor kRed = RGBWColor.fromHex("rgba(255, 0, 0, 1)").orElseThrow();
    public static final RGBWColor kBlue = RGBWColor.fromHex("rgba(0, 102, 255, 1)").orElseThrow();
    public static final RGBWColor kYellow = RGBWColor.fromHex("rgba(217, 217, 0, 1)").orElseThrow();
    public static final RGBWColor kWhite = RGBWColor.fromHex("rgba(217, 217, 217, 1)").orElseThrow();
    public static final RGBWColor kPink = RGBWColor.fromHex("rgba(236, 16, 139, 1)").orElseThrow();

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
        DEFAULT(new SingleFadeAnimation(0, 37).withColor(kPink).withFrameRate(1)),
        DISABLED(new SolidColor(0, 37).withColor(kPink)),
        DEFENSE(new SolidColor(0, 37).withColor(kBlue)),
        TRENCH(new SolidColor(0, 37).withColor(kWhite)),
        SHOOTINTAKE(new StrobeAnimation(0, 37).withColor(kPink).withFrameRate(10)),
        INTAKE(new StrobeAnimation(0, 37).withColor(kGreen).withFrameRate(10)),
        HERD(new SolidColor(0, 37).withColor(kYellow)),
        WARNING(new StrobeAnimation(0, 37).withColor(kRed).withFrameRate(5));

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
