package frc.robot.subsystems.Hopper;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Hopper.HopperConstants.XboxControllerConstants;

public class HopperSubsystem {

    private final CommandXboxController m_driverController = new CommandXboxController(XboxControllerConstants.kDriverControllerPort);
    private final SparkMax Hoppermotor1 = new SparkMax (HopperConstants.Hoppermotor.Hoppermotor1, MotorType.kBrushless);
    private final SparkMax Hoppermotor2 = new SparkMax (HopperConstants.Hoppermotor.Hoppermotor2, MotorType.kBrushless);
    
    public Command Hoppermotor1cm(double speed) {
        return run(() -> Hoppermotor1.set(speed))
            .andThen(() -> Hoppermotor1.stopMotor()); // Safety: Stop the motors when time is up!
    }
    public Command Hoppermotor2cm(double speed) {
        return run(() -> Hoppermotor2.set(speed))
            .andThen(() -> Hoppermotor2.stopMotor()); // Safety: Stop the motors when time is up!
    }
     public Command HopperBothcm(double speed) {
        return run(() -> {
            Hoppermotor1.set(speed);
            Hoppermotor2.set(speed);
        }).andThen(() -> {
            Hoppermotor1.stopMotor();
            Hoppermotor2.stopMotor();
        }); // Safety: Stop the motors when time is up!
    }
}

