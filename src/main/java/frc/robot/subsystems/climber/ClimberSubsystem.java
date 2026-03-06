package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX climberMotor = new TalonFX(ClimberConstants.kClimberMotorId);
    private boolean isClimbing;

    
    public ClimberSubsystem() {

        new ProfiledPIDController(
            ClimberConstants.kPClimb.get(), 
            ClimberConstants.kIClimb.get(), 
            ClimberConstants.kDClimb.get(),
            new TrapezoidProfile.Constraints(ClimberConstants.kVClimb.get(), 
                                            ClimberConstants.kAClimb.get()));
        }

    public boolean isClimbing(){
        return this.isClimbing;   
    }

    public void stop( ) {
        climberMotor.stopMotor();
        this.isClimbing = false;
    }
    
    public void setClimberSpeed(double speed) {
        climberMotor.set(speed);
        this.isClimbing = true;
    }

    public Command climbCommand(double speed) {
        return runEnd(() -> setClimberSpeed(speed), this::stop);
    }

    public Command runClimbCommand() {
        return runEnd(() -> setClimberSpeed(0), this::stop); // This is where Alex stopped on Thursday, 03/05.
    }

}
