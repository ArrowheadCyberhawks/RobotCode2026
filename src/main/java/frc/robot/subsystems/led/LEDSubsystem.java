package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.led.LEDConstants.*;

/**
 * Subsystem wrapper around a CTRE CANdle LED controller.
 *
 * This is a direct port of the standalone CANdle TimedRobot example into
 * a command-based {@link SubsystemBase}. The animation selection and state
 * changes are handled in {@link #periodic()}.
 */
public class LEDSubsystem extends SubsystemBase {

    private final CANdle candle = new CANdle(kCANdleId, kCANBus);

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

    private AnimationType anim0State = AnimationType.None;
    private AnimationType anim1State = AnimationType.None;

    private final SendableChooser<AnimationType> anim0Chooser = new SendableChooser<>();
    private final SendableChooser<AnimationType> anim1Chooser = new SendableChooser<>();

    public LEDSubsystem() {
        // Configure CANdle
        var cfg = new CANdleConfiguration();
        // set the LED strip type and brightness
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;
        // disable status LED when being controlled
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        candle.getConfigurator().apply(cfg);

        // clear all previous animations
        for (int i = 0; i < 8; ++i) {
            candle.setControl(new EmptyAnimation(i));
        }
        // set the onboard LEDs to a solid color
        candle.setControl(new SolidColor(0, 3).withColor(kGreen));
        candle.setControl(new SolidColor(4, 7).withColor(kWhite));

        // add animations to chooser for slot 0
        anim0Chooser.setDefaultOption("Color Flow", AnimationType.ColorFlow);
        anim0Chooser.addOption("Rainbow", AnimationType.Rainbow);
        anim0Chooser.addOption("Twinkle", AnimationType.Twinkle);
        anim0Chooser.addOption("Twinkle Off", AnimationType.TwinkleOff);
        anim0Chooser.addOption("Fire", AnimationType.Fire);

        // add animations to chooser for slot 1
        anim1Chooser.setDefaultOption("Larson", AnimationType.Larson);
        anim1Chooser.addOption("RGB Fade", AnimationType.RgbFade);
        anim1Chooser.addOption("Single Fade", AnimationType.SingleFade);
        anim1Chooser.addOption("Strobe", AnimationType.Strobe);
        anim1Chooser.addOption("Fire", AnimationType.Fire);

        SmartDashboard.putData("Animation 0", anim0Chooser);
        SmartDashboard.putData("Animation 1", anim1Chooser);
    }

    @Override
    public void periodic() {
        // Slot 0 animation selection
        final var anim0Selection = anim0Chooser.getSelected();
        if (anim0State != anim0Selection) {
            anim0State = anim0Selection;

            switch (anim0State) {
                default:
                case ColorFlow:
                    candle.setControl(
                            new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                                    .withColor(kViolet));
                    break;
                case Rainbow:
                    candle.setControl(
                            new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0));
                    break;
                case Twinkle:
                    candle.setControl(
                            new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                                    .withColor(kViolet));
                    break;
                case TwinkleOff:
                    candle.setControl(
                            new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                                    .withColor(kViolet));
                    break;
                case Fire:
                    candle.setControl(
                            new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0));
                    break;
                case None:
                    candle.setControl(new EmptyAnimation(0));
                    break;
            }
        }

        final var anim1Selection = anim1Chooser.getSelected();
        if (anim1State != anim1Selection) {
            anim1State = anim1Selection;

            switch (anim1State) {
                default:
                case Larson:
                    candle.setControl(
                            new LarsonAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                                    .withColor(kRed));
                    break;
                case RgbFade:
                    candle.setControl(
                            new RgbFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1));
                    break;
                case SingleFade:
                    candle.setControl(
                            new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                                    .withColor(kRed));
                    break;
                case Strobe:
                    candle.setControl(
                            new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                                    .withColor(kRed));
                    break;
                case Fire:
                    candle.setControl(
                            new FireAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                                    .withDirection(AnimationDirectionValue.Backward)
                                    .withCooling(0.4)
                                    .withSparking(0.5));
                    break;
                case None:
                    candle.setControl(new EmptyAnimation(1));
                    break;
            }
        }
    }
}
