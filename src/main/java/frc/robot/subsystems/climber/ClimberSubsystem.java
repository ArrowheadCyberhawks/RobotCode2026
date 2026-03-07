package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX climberMotor = new TalonFX(ClimberConstants.kClimberMotorId);
    //stop climber motor
    public void stopMotor() {
        climberMotor.stopMotor();
    }
    
    //set climber speed 
    public void setClimberSpeed(double speed) {
        climberMotor.set(speed);
    }

    //run climber at given speed
    public Command climbCommand(double speed) {
        return runEnd(() -> setClimberSpeed(speed), this::stopMotor);
    }
    //run climber in reverse
    public Command runClimbdown() {
        return runEnd(() -> setClimberSpeed(-0.5), this::stopMotor);
    }
    //run Climber forward
    public Command runClimberup() {
        return runEnd(() -> setClimberSpeed(0.5), this::stopMotor);
    }
}

