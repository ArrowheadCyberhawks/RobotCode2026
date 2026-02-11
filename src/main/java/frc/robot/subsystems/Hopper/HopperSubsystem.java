package frc.robot.subsystems.Hopper;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class HopperSubsystem {

    private final SparkMax hopperMotor1 = new SparkMax (HopperConstants.hopperMotor.hopperMotor1, MotorType.kBrushless);
    private final SparkMax hopperMotor2 = new SparkMax (HopperConstants.hopperMotor.hopperMotor2, MotorType.kBrushless);
    /**
     * Command runs hopper motor 1 at given speed
     * @param speed
     * @return
     */

    public Command runHopperMotor1(double speed) {
        return run(() -> hopperMotor1.set(speed))
            .finallyDo(() -> hopperMotor1.stopMotor()); // Safety: Stop the motors when time is up!
    }
    /**
     * Command runs hopper motor 2 at given speed
     * @param speed
     * @return
     */

    public Command runHopperMotor2(double speed) {
        return run(() -> hopperMotor2.set(speed))
            .finallyDo(() -> hopperMotor2.stopMotor()); // Safety: Stop the motors when time is up!
    }
    /**
     * Command runs both hopper motors at given speed
     * @param speed
     * @return
     */
    public Command runHopperMotors(double speed) {
        return new ParallelCommandGroup(
            runHopperMotor1(speed),
            runHopperMotor2(speed)
        );
    }
}

