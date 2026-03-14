package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ClimberSubsystem
 *
 * Provides low-level control for a two-motor climber using CTRE Phoenix6
 * TalonFX controllers. This class configures the motors and exposes simple
 * methods to move the climber using Motion Magic (position) and velocity
 * control. Both motors are commanded together here so higher-level commands
 * don't need to duplicate logic.
 */

 /** Side note: we currently need to rewrite some of the code. Our climber only uses one motor
  *  (CAN ID 30), so we need to either remove the second motor code or rewrite it so it is irrelevent. 
  TLDR; keep kleft motor, rewrite/remove right motor.
  */
public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX leftMotor = new TalonFX(ClimberConstants.kLeftMotorId);
   // private final TalonFX rightMotor = new TalonFX(ClimberConstants.kRightMotorId);
    private final MotionMagicVoltage climbRequest = new MotionMagicVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final NeutralOut neutralOut = new NeutralOut();

    public ClimberSubsystem() {
        configureMotor(leftMotor);
        //configureMotor(rightMotor);
        // Zero encoders at startup
        leftMotor.setPosition(0.0); //need to fact check
        //rightMotor.setPosition(0.0);
    }

    private void configureMotor(TalonFX motor) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Slot0.kP = ClimberConstants.kPClimb;
        config.Slot0.kI = ClimberConstants.kIClimb;
        config.Slot0.kD = ClimberConstants.kDClimb;
        config.Slot0.kG = ClimberConstants.kGClimb;
        config.Slot0.kV = ClimberConstants.kVClimb;
        config.Slot0.kA = ClimberConstants.kAClimb;

        config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.kClimbCruiseRps;
        config.MotionMagic.MotionMagicAcceleration = ClimberConstants.kClimbAccelRps2;

        motor.getConfigurator().apply(config);
    }

    /**
     * Move both climber motors to an absolute position (motor rotations) using Motion Magic.
     * @param motorRotations absolute motor rotations (motor-side units)
     */
    public void moveToMotorRotations(double motorRotations) {
        leftMotor.setControl(climbRequest.withPosition(motorRotations));
       // rightMotor.setControl(climbRequest.withPosition(motorRotations));
    }

    /**
     * Move the climber by a delta (relative) in motor rotations.
     */
    public void moveByMotorRotations(double deltaRotations) {
        double current = getMotorRotations();
        moveToMotorRotations(current + deltaRotations);
    }

    /**
     * Convenience: move using climber-side rotations (mechanism rotations).
     * Converts mechanism rotations -> motor rotations using gear ratio.
     */
    public void moveToClimberRotations(double climberRotations) {
        double motorRot = climberRotations * ClimberConstants.kClimberGearRatio;
        moveToMotorRotations(motorRot);
    }

    /**
     * Set both motors to a velocity (motor RPS).
     */
    public void setVelocityRps(double motorRps) {
        leftMotor.setControl(velocityRequest.withVelocity(motorRps));
      //  rightMotor.setControl(velocityRequest.withVelocity(motorRps));
    }

    /**
     * Stop both motors (neutral output / brake behavior will hold position).
     */



    
    public void stop() {
        leftMotor.setControl(neutralOut);
     //   rightMotor.setControl(neutralOut);
    }

    /**
     * Reset both encoder positions to zero (motor rotations).
     */
    public void resetEncoders() {
        leftMotor.setPosition(0.0);
        //rightMotor.setPosition(0.0);
    }

    /**
     * Return the average motor rotations of the two climber motors.
     */
    public double getMotorRotations() {
        double left = leftMotor.getPosition().getValueAsDouble();
        //double right = rightMotor.getPosition().getValueAsDouble();
        return (left) / 2.0; //alternative to needing to keep right motor in code
        //return (left + right) / 2.0;
    }

    @Override
    public void periodic() {
        // Publish a few useful diagnostics
        SmartDashboard.putNumber("Climber/MotorRotations", getMotorRotations());
    }

    
//next step is to set up the maximum height (zero out climber before turning robot on)
}
 