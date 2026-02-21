package frc.robot.subsystems.hopper;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {

    private final SparkMax hopperMotor;
    private final SparkMax kickerMotor;

    private final SparkClosedLoopController hopperPIDController;
    private final SparkClosedLoopController kickerPIDController;

    private final RelativeEncoder hopperEncoder;
    private final RelativeEncoder kickerEncoder;

    private SparkMaxConfig hopperConfig;
    private SparkMaxConfig kickerConfig;

    private HopperState hopperState = HopperState.IDLE;

    private double hopperTargetRpm = 0.0;
    private double kickerTargetRpm = 0.0;

        public HopperSubsystem()    {
            //create motors
            hopperMotor = new SparkMax(HopperConstants.hopperMotor.hopperMotor, MotorType.kBrushless);
            kickerMotor = new SparkMax(HopperConstants.hopperMotor.kickerMotor, MotorType.kBrushless);

            //Get encoders
            hopperEncoder = hopperMotor.getEncoder();
            kickerEncoder = kickerMotor.getEncoder();

            //Get closed-loop controllers
            hopperPIDController = hopperMotor.getClosedLoopController();
            kickerPIDController = kickerMotor.getClosedLoopController();

            //create configs
            hopperConfig = new SparkMaxConfig();
            kickerConfig = new SparkMaxConfig();
        
            //configure motors
            configureHopper();
            configureKicker();
        }
    //State machine for hopper and kicker control
    public enum HopperState {
        IDLE,   //Both motors off
        ON      //Both motors running at given speed
    }

     /**Set the current hopper state*/
    public void setHopperState(HopperState state) {
        this.hopperState = state;
    }

    /**Read the current hopper state*/
    public HopperState getHopperState() {
        return this.hopperState;

    }

    //Hopper motor configuration
    private void configureHopper() {
        hopperConfig
            .idleMode(IdleMode.kCoast)
            .inverted(true);

        // Configure PID and feedforward for velocity control
        hopperConfig.closedLoop
            .pid(HopperConstants.kPHopper.get(), 
                 HopperConstants.kIHopper.get(), 
                 HopperConstants.kDHopper.get())
            .feedForward
                .kV(HopperConstants.kVHopper.get())
                .kS(HopperConstants.kSHopper.get());

        hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    //Kicker motor configuration
    private void configureKicker() {
        kickerConfig
            .idleMode(IdleMode.kCoast)
            .inverted(false);

        // Configure PID and feedforward for velocity control
        kickerConfig.closedLoop
            .pid(HopperConstants.kPKicker.get(), 
                 HopperConstants.kIKicker.get(), 
                 HopperConstants.kDKicker.get())
            .feedForward
                .kV(HopperConstants.kVKicker.get())
                .kS(HopperConstants.kSKicker.get());

        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // function for hopper and givern speed
    public void runHopper() {
        hopperTargetRpm = HopperConstants.kHopperRpm.get();
    }
    // function for kicker at given speed
    public void runKicker() {
        kickerTargetRpm = HopperConstants.kKickerRpm.get();
    }
    // function for hopper in reverse at given speed to fix jams
    public void reverseHopper() {
        hopperTargetRpm = HopperConstants.kHopperReversedRpm.get();
    }
    // function for kicker in reverse at given speed to fix jams
    public void reverseKicker() {
        kickerTargetRpm = HopperConstants.KickerReverseRpm.get();
    }

    // function for stops hopper
    public void stopHopper() {
        hopperTargetRpm = 0.0;
        hopperMotor.stopMotor();
    }

    // function for stops kicker
    public void stopKicker() {
        kickerTargetRpm = 0.0;
        kickerMotor.stopMotor();
    }

    @Override
    public void periodic() {
        switch (hopperState) {
            case IDLE:
                //Runs both motors at 0 RPM (OFF)
                stopHopper();
                stopKicker();
                break;
            case ON:
                //Runs both motors at given speed
                runHopper();
                runKicker();
                break;
        }
    
        Logger.recordOutput("Hopper/State", hopperState.name());

        // Update motor control and PID parameters
        updateHopperControl();
        updateKickerControl();
        updateHopperPID();
        updateKickerPID();
    }


    /**updates the hopper motor control using REV closed-loop velocity PID*/
    private void updateHopperControl() {
        if (Math.abs(hopperTargetRpm) > 10) {
            // Convert RPM to rotations per second for the PID controller
            double targetRps = hopperTargetRpm / 60.0;
            hopperPIDController.setReference(targetRps, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        } else {
            hopperMotor.stopMotor();
        }

        // Log telemetry
        Logger.recordOutput("Hopper/TargetRPM", hopperTargetRpm);
        Logger.recordOutput("Hopper/ActualRPM", hopperEncoder.getVelocity());
    }

    /**updates the kicker motor control using REV closed-loop velocity PID*/
    private void updateKickerControl() {
        if (Math.abs(kickerTargetRpm) > 10) {
            // Convert RPM to rotations per second for the PID controller
            double targetRps = kickerTargetRpm / 60.0;
            kickerPIDController.setReference(targetRps, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        } else {
            kickerMotor.stopMotor();
        }

        // Log telemetry
        Logger.recordOutput("Kicker/TargetRPM", kickerTargetRpm);
        Logger.recordOutput("Kicker/ActualRPM", kickerEncoder.getVelocity());
    }
    
    /**updates the hopper motor PID parameters if they have changed*/
    private void updateHopperPID() {
        int id = this.hashCode();
        if (HopperConstants.kPHopper.hasChanged(id) || 
            HopperConstants.kIHopper.hasChanged(id) ||
            HopperConstants.kDHopper.hasChanged(id) ||
            HopperConstants.kVHopper.hasChanged(id) ||
            HopperConstants.kSHopper.hasChanged(id)) {

            hopperConfig.closedLoop
                .pid(HopperConstants.kPHopper.get(), 
                     HopperConstants.kIHopper.get(), 
                     HopperConstants.kDHopper.get())
                .feedForward
                    .kV(HopperConstants.kVHopper.get())
                    .kS(HopperConstants.kSHopper.get());
            
            hopperMotor.configure(hopperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    /**updates the kicker motor PID parameters if they have changed*/
    private void updateKickerPID() {
        int id = this.hashCode();
        if (HopperConstants.kPKicker.hasChanged(id) || 
            HopperConstants.kIKicker.hasChanged(id) ||
            HopperConstants.kDKicker.hasChanged(id) ||
            HopperConstants.kVKicker.hasChanged(id) ||
            HopperConstants.kSKicker.hasChanged(id)) {

            kickerConfig.closedLoop
                .pid(HopperConstants.kPKicker.get(), 
                     HopperConstants.kIKicker.get(), 
                     HopperConstants.kDKicker.get())
                .feedForward
                    .kV(HopperConstants.kVKicker.get())
                    .kS(HopperConstants.kSKicker.get());
            
            kickerMotor.configure(kickerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

}
/*public class HopperSubsystem {

    private final SparkMax hopperMotor1 = new SparkMax (HopperConstants.hopperMotor.hopperMotor1, MotorType.kBrushless);
    private final SparkMax hopperMotor2 = new SparkMax (HopperConstants.hopperMotor.hopperMotor2, MotorType.kBrushless);
    /**
     * Command runs hopper motor 1 at given speed
     * @param speed
     * @return
     */
/* 
    public Command runHopperMotor1(double speed) {
        return run(() -> hopperMotor1.set(speed))
            .finallyDo(() -> hopperMotor1.stopMotor()); // Safety: Stop the motors when time is up!
    }
    /**
     * Command runs hopper motor 2 at given speed
     * @param speed
     * @return
     */

/*     public Command runHopperMotor2(double speed) {
        return run(() -> hopperMotor2.set(speed))
            .finallyDo(() -> hopperMotor2.stopMotor()); // Safety: Stop the motors when time is up!
    }
    /**
     * Command runs both hopper motors at given speed
     * @param speed
     * @return
     */
/*     public Command runHopperMotors(double speed) {
        return new ParallelCommandGroup(
            runHopperMotor1(speed),
            runHopperMotor2(speed)
        );
    }
}
*/



