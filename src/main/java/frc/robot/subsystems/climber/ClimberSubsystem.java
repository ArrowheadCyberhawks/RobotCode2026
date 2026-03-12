package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX climberMotor;


        public ClimberSubsystem() {
            climberMotor = new TalonFX(ClimberConstants.kClimberMotorId);
            configureClimber();
        }

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

    private void configureClimber() {
            TalonFXConfiguration climberconfig = new TalonFXConfiguration();

            climberconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            climberconfig.Feedback.SensorToMechanismRatio =  ClimberConstants.kClimberGearRatio;

            climberconfig.Slot0.kP = ClimberConstants.kPClimb.get();
            climberconfig.Slot0.kI = ClimberConstants.kIClimb.get();
            climberconfig.Slot0.kD = ClimberConstants.kPClimb.get();
            climberconfig.Slot0.kA = ClimberConstants.kAClimb.get();
            climberconfig.Slot0.kV = ClimberConstants.kVClimb.get();
            climberconfig.Slot0.kG = ClimberConstants.kGClimb.get();
            
            climberMotor.getConfigurator().apply(climberconfig);
    }

      @Override
  public void periodic() {
    // Update PID values from LoggedTunableNumbers
    if (ClimberConstants.kPClimb.hasChanged(hashCode()) || 
        ClimberConstants.kIClimb.hasChanged(hashCode()) || 
        ClimberConstants.kDClimb.hasChanged(hashCode()) ||
        ClimberConstants.kVClimb.hasChanged(hashCode()) ||
        ClimberConstants.kAClimb.hasChanged(hashCode()) ||
        ClimberConstants.kGClimb.hasChanged(hashCode())) {
      // Reconfigure PID on the motor controller
      Slot0Configs slot0 = new Slot0Configs();
      slot0.kP = ClimberConstants.kPClimb.get();
      slot0.kI = ClimberConstants.kIClimb.get();
      slot0.kD = ClimberConstants.kDClimb.get();
      slot0.kV = ClimberConstants.kVClimb.get();
      slot0.kA = ClimberConstants.kAClimb.get();
      slot0.kG = ClimberConstants.kGClimb.get();
      climberMotor.getConfigurator().apply(slot0);
        }
    }
}

