package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.intake.IntakeConstants.IntakePosition;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX pivotMotor = new TalonFX(IntakeConstants.kPivotMotorId);
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.kRollerMotorId);
    private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);
    private final VelocityVoltage rollerRequest = new VelocityVoltage(0);

    public IntakeSubsystem() {
        configurePivot();
        configureRoller();
        pivotMotor.setPosition(degreesToMotorRotations(0.0));
    }


    private void configurePivot() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.kP = IntakeConstants.kPPivot;
    config.Slot0.kI = IntakeConstants.kIPivot;
    config.Slot0.kD = IntakeConstants.kDPivot;
    config.Slot0.kG = IntakeConstants.kGPivot;
    config.Slot0.kV = IntakeConstants.kVPivot;
    config.Slot0.kA = IntakeConstants.kAPivot;

    config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.kPivotCruiseRps;
    config.MotionMagic.MotionMagicAcceleration = IntakeConstants.kPivotAccelRps2;

        pivotMotor.getConfigurator().apply(config);
    }

    private void configureRoller() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.Slot0.kP = IntakeConstants.kPRoller;
    config.Slot0.kI = IntakeConstants.kIRoller;
    config.Slot0.kD = IntakeConstants.kDRoller;
    config.Slot0.kV = IntakeConstants.kVRoller;

        rollerMotor.getConfigurator().apply(config);
    }

    public void moveToPosition(IntakePosition position) {
        double motorRotations = degreesToMotorRotations(position.degrees);
        pivotMotor.setControl(pivotRequest.withPosition(motorRotations));
    }

    public double getPivotDegrees() {
        return motorRotationsToDegrees(pivotMotor.getPosition().getValueAsDouble());
    }

    public void runIntake() {
    rollerMotor.setControl(rollerRequest.withVelocity(IntakeConstants.kIntakeRps));
    }

    public void runOuttake() {
    rollerMotor.setControl(rollerRequest.withVelocity(IntakeConstants.kOuttakeRps));
    }

    public void stopRoller() {
        rollerMotor.set(0.0);
    }

    private double degreesToMotorRotations(double degrees) {
    return (degrees / 360.0) * IntakeConstants.kPivotGearRatio;
    }

    private double motorRotationsToDegrees(double motorRotations) {
    return (motorRotations / IntakeConstants.kPivotGearRatio) * 360;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Pivot Degrees", getPivotDegrees());
    }
}
