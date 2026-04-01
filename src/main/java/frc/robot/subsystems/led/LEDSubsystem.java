package frc.robot.subsystems.led;

import java.util.function.Supplier;

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
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.led.LEDConstants.LEDState;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;

import static frc.robot.subsystems.led.LEDConstants.*;

/**
 * Subsystem wrapper around a CTRE CANdle LED controller.
 *
 * This is a direct port of the standalone CANdle TimedRobot example into
 * a command-based {@link SubsystemBase}. The animation selection and state
 * changes are handled in {@link #periodic()}.
 */
public class LEDSubsystem extends SubsystemBase {

    private final CANdle candle = new CANdle(kCANdleId);

    private final Supplier<IntakeState> intakeStateSupplier;
    private final Supplier<ShooterState> shooterStateSupplier;

    public LEDState state;

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

    public LEDSubsystem(Supplier<IntakeState> intakeStateSupplier, Supplier<ShooterState> shooterStateSupplier) {
        this.intakeStateSupplier = intakeStateSupplier;
        this.shooterStateSupplier = shooterStateSupplier;
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
        candle.setControl(new SolidColor(0, 37).withColor(LEDState.DEFAULT.getColor()));
    }

    @Override
    public void periodic() {
        IntakeState intakeState = intakeStateSupplier.get();
        ShooterState shooterState = shooterStateSupplier.get();

        if (shooterState == ShooterState.TRENCH) {
            setState(state);LEDState.TRENCH.getColor();
        } else if (intakeState == IntakeState.RUN) {
            if (shooterState == ShooterState.AIM) {
                setState(LEDState.SHOOTINTAKE);
            } else {
                setState(LEDState.INTAKE);
            }
        } else if (intakeState == IntakeState.REVERSE) {
            setState(LEDState.HERD);
        } else if (intakeState == IntakeState.STOW) {
            setState(LEDState.DEFENSE);
        } else {
            setState(LEDState.DEFAULT);
        }

        candle.setControl(new SolidColor(0, 99).withColor(state.getColor()));
    }

    public void setState(LEDState state) {
        this.state = state;
    }
}
