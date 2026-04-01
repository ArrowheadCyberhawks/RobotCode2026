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
import frc.robot.subsystems.intake.IntakeConstants.IntakeState;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;
import java.util.function.Supplier;

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

    private final Supplier<IntakeState> intakeStateSupplier;
    private final Supplier<ShooterState> shooterStateSupplier;
    LEDState state = LEDState.DEFAULT;


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

        candle.setControl(new SolidColor(0, 399).withColor(LEDState.DEFAULT.color));
    }


    @Override
    public void periodic() {
        
        if (intakeStateSupplier.get() == IntakeState.RUN) {
            if (shooterStateSupplier.get() == ShooterState.AIM) {
                setLEDState(LEDState.INTAKESHOOT);
            } else {
                setLEDState(LEDState.INTAKE);
            }
        } else if (intakeStateSupplier.get() == IntakeState.REVERSE) {
            setLEDState(LEDState.HERD);
        }
        if (shooterStateSupplier.get() == ShooterState.AIM) {
            setLEDState(LEDState.SHOOTING);
        } 
        if (intakeStateSupplier.get() == IntakeState.IDLE) {
            setLEDState(LEDState.DEFENSE);
        }
        if (intakeStateSupplier.get() == IntakeState.IDLE) {
            setLEDState(LEDState.DEFAULT);
        }        
        
        candle.setControl(new SolidColor(0, 8).withColor(state.color));
    }
    
    public void setLEDState(LEDState state) {
        this.state = state;
    }
    
}



