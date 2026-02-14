package frc.robot.subsystems.hopper;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {

    private final SparkMax hopperMotor;
    private final SparkMax kickerMotor;

    private final ProfiledPIDController hopperController;
    private final ProfiledPIDController kickerController;

    private SparkMaxConfig hopperConfig;
    private SparkMaxConfig kickerConfig;

    private HopperState hopperState = HopperState.IDLE;

    private double hopperTargetRpm = 0.0;
    private double kickerTargetRpm = 0.0;

        public HopperSubsystem()    {
            //create motors
            hopperMotor = new SparkMax(HopperConstants.hopperMotor.hopperMotor, MotorType.kBrushless);
            kickerMotor = new SparkMax(HopperConstants.hopperMotor.kickerMotor, MotorType.kBrushless);

            //create configs
            hopperConfig = new SparkMaxConfig();
            kickerConfig = new SparkMaxConfig();

            //create PID controllers for hopper motor with trapezoidal motion profile
            hopperController = new ProfiledPIDController(
            HopperConstants.kPHopper.get(),
            HopperConstants.kIHopper.get(),
            HopperConstants.kDHopper.get(),
            new TrapezoidProfile.Constraints(
                HopperConstants.kHopperMaxVelocityRps.get(),
                HopperConstants.kHopperMaxAccelRps2.get()
            )
        );

            //create PID controllers for kicker motor with trapezoidal motion profile
            kickerController = new ProfiledPIDController(
                HopperConstants.kPKicker.get(),
                HopperConstants.kIKicker.get(),
                HopperConstants.kDKicker.get(),
                new TrapezoidProfile.Constraints(
                    HopperConstants.kKickerMaxVelocityRps.get(),
                    HopperConstants.kKickerMaxAccelRps2.get()
                )
            );
        
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
            .inverted ( false );

    hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    //Kicker motor configuration
    private void configureKicker() {
        kickerConfig
            .idleMode(IdleMode.kCoast)
            .inverted ( false );

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
    
        SmartDashboard.putString("Hopper/State", hopperState.name());

        // Update motor control and PID parameters
    updateHopperControl();
    updateKickerControl();
    updateHopperPID();
    updateKickerPID();
    }




    /**updates the hopper motor control with simple voltage-based speed control*/
    private void updateHopperControl() {
   
        if (hopperTargetRpm > 10) {
    
            double maxRPM = 5000.0;
            double voltage = (hopperTargetRpm / maxRPM) * 12.0;
            voltage = Math.max(-12.0, Math.min(12.0, voltage));
            hopperMotor.setVoltage(voltage);
        } else {
            hopperMotor.stopMotor();
        }
    }

    /**updates the kicker motor control with simple voltage-based speed control*/
    private void updateKickerControl() {
   
        if (kickerTargetRpm > 10) {
    
            double maxRPM = 5000.0;
            double voltage = (kickerTargetRpm / maxRPM) * 12.0;
            voltage = Math.max(-12.0, Math.min(12.0, voltage));
            kickerMotor.setVoltage(voltage);
        } else {
            kickerMotor.stopMotor();
        }
    }
    
    /**updates the hopper motor PID parameters if they have changed*/
    private void updateHopperPID() {
        int id = this.hashCode();
        if (HopperConstants.kPHopper.hasChanged(id) || HopperConstants.kIHopper.hasChanged(id) ||
            HopperConstants.kDHopper.hasChanged(id) || HopperConstants.kHopperMaxVelocityRps.hasChanged(id) ||
            HopperConstants.kHopperMaxAccelRps2.hasChanged(id)) {


            hopperController.setP(HopperConstants.kPHopper.get());
            hopperController.setI(HopperConstants.kIHopper.get());
            hopperController.setD(HopperConstants.kDHopper.get());
            hopperController.setConstraints(
                new TrapezoidProfile.Constraints(
                    HopperConstants.kHopperMaxVelocityRps.get(),
                    HopperConstants.kHopperMaxAccelRps2.get()
                )
            
            );
        }
    }
    /**updates the kicker motor PID parameters if they have changed*/
    private void updateKickerPID() {
        int id = this.hashCode();
        if (HopperConstants.kPKicker.hasChanged(id) || HopperConstants.kIKicker.hasChanged(id) ||
            HopperConstants.kDKicker.hasChanged(id) || HopperConstants.kKickerMaxVelocityRps.hasChanged(id) ||
            HopperConstants.kKickerMaxAccelRps2.hasChanged(id)) {


                kickerController.setP(HopperConstants.kPKicker.get());
                kickerController.setI(HopperConstants.kIKicker.get());
                kickerController.setD(HopperConstants.kDKicker.get());
                kickerController.setConstraints(
                    new TrapezoidProfile.Constraints(
                        HopperConstants.kKickerMaxVelocityRps.get(),
                        HopperConstants.kKickerMaxAccelRps2.get()
                    )
            
            );
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




