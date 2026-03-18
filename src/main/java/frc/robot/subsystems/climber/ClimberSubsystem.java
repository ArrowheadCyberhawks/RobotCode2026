package frc.robot.subsystems.climber;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.InvertedValue;
// Imports documents

public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX climberMotor;

        public ClimberSubsystem() {
            climberMotor = new TalonFX(ClimberConstants.kClimberMotorId); //defining term climberMotor using the motor ID from the climber constants
            configureClimber();
        }
        //  Defines a starting position for the climber.
         

        // Stops the climber motor
    public void stopMotor() {
        climberMotor.stopMotor();
    }

    // Gets the climber position
    public double getClimberPosition() {
        return climberMotor.getPosition().getValueAsDouble();
    }

    // Resets the climber position
    public void resetClimberPosition(double rotations) {
        climberMotor.setPosition(rotations);
    }

    // Sets the climber speed 
    public void setClimberMotor(double speed) {
        climberMotor.set(speed);
    }

    // Runs climber at given speed
    public Command climbCommand(Supplier<Double> speed) {
        return runEnd(() -> setClimberMotor(speed.get()), this::stopMotor);
    }
    // Runs climber in reverse - climb down
    public Command runClimberDown() {
        return climbCommand(() -> ClimberConstants.climberDownSpeed.get()).until(
            () -> getClimberPosition() <= ClimberConstants.kClimberMinPosition.get());
    }
    //run Climber forward - climb up
    public Command runClimberUp() {
        return climbCommand(() -> ClimberConstants.climberUpSpeed.get()).until(
            () -> getClimberPosition() >= ClimberConstants.kClimberMaxPosition.get());
    }
    //sets up code that allows us to tell the motor how fast it should move and in which direction.


    // Configures the climber motor with appropriate settings for control and feedback
    private void configureClimber() {
            TalonFXConfiguration climberConfig = new TalonFXConfiguration();

            climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; //sets up a brake

            climberConfig.Feedback.SensorToMechanismRatio =  ClimberConstants.kClimberGearRatio;

            // Configures the PID values for the climber motor
            climberConfig.Slot0.kP = ClimberConstants.kPClimb.get();
            climberConfig.Slot0.kI = ClimberConstants.kIClimb.get();
            climberConfig.Slot0.kD = ClimberConstants.kDClimb.get();
            climberConfig.Slot0.kA = ClimberConstants.kAClimb.get();
            climberConfig.Slot0.kV = ClimberConstants.kVClimb.get();
            climberConfig.Slot0.kG = ClimberConstants.kGClimb.get();
            // Inverts the motor
            
            // Set limits in rotations
            climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimberConstants.kClimberMaxPosition.get(); 
            climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimberConstants.kClimberMinPosition.get(); 

            // Enable the limits
            climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

            climberConfig.CurrentLimits.SupplyCurrentLimit = 40.0; // Amps
            climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            climberConfig.CurrentLimits.StatorCurrentLimit = 40.0; // Amps
            climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;

            climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
           
            /* Positive values will pull the robot up,
            negative values will let the robot down.
            Testing the climber by itself will make it seem like the 
            positive values is making the climber go down. This is 
            what we want it to do. */
            
            climberMotor.getConfigurator().apply(climberConfig);
    }

      @Override
  public void periodic() {
    // Update PID values from LoggedTunableNumbers and reconfigures on the motor controller
    if (ClimberConstants.kPClimb.hasChanged(hashCode()) || 
        ClimberConstants.kIClimb.hasChanged(hashCode()) || 
        ClimberConstants.kDClimb.hasChanged(hashCode()) ||
        ClimberConstants.kVClimb.hasChanged(hashCode()) ||
        ClimberConstants.kAClimb.hasChanged(hashCode()) ||
        ClimberConstants.kGClimb.hasChanged(hashCode()) ||
        ClimberConstants.kClimberMaxPosition.hasChanged(hashCode()) ||
        ClimberConstants.kClimberMinPosition.hasChanged(hashCode())) {

      Slot0Configs slot0 = new Slot0Configs();
      // Sets up the PID values for the climber motor
      slot0.kP = ClimberConstants.kPClimb.get();
      slot0.kI = ClimberConstants.kIClimb.get();
      slot0.kD = ClimberConstants.kDClimb.get();
      slot0.kV = ClimberConstants.kVClimb.get();
      slot0.kA = ClimberConstants.kAClimb.get();
      slot0.kG = ClimberConstants.kGClimb.get();
        // Set limits in rotations
      
      climberMotor.getConfigurator().apply(slot0);
    }
    Logger.recordOutput("Climber/Position", getClimberPosition());
    }
}
