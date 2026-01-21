// package frc.robot.subsystems.intake;

// import static edu.wpi.first.units.Units.Meters;
// import static edu.wpi.first.units.Units.Volts;
// import static frc.robot.Constants.IntakeConstants.*;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj.util.Color;
// import java.util.function.Supplier;

// /** Intake subsystem without visualizer and with code-only controller. */
// public class IntakeSubsystem extends SubsystemBase {
//     private final IntakeIO pivotIO;
//     private final IntakeIO rightIO;

//     private final IntakeIOInputsAutoLogged pivotInputs = new IntakeIOInputsAutoLogged();
//     private final IntakeIOInputsAutoLogged rightInputs = new IntakeIOInputsAutoLogged();

//     private final ProfiledPIDController pivotController;
//     private final ProfiledPIDController rightController;

//     private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

//     private boolean isDeployed = true;

//     private final Trigger deploypivotTrigger;
//     private final Trigger deployRightTrigger;

//     /** Hard-coded PID + trapezoid constraints */
//     private static final double kP = 5.0;
//     private static final double kI = 0.0;
//     private static final double kD = 0.5;
//     private static final double kMaxVel = 2.0; // meters/sec
//     private static final double kMaxAcc = 1.0; // meters/sec^2

//     public IntakeSubsystem(IntakeIO pivotIO, IntakeIO rightIO, Supplier<ChassisSpeeds>
// chassisSpeedsSupplier) {
//         this.pivotIO = pivotIO;
//         this.rightIO = rightIO;
//         this.chassisSpeedsSupplier = chassisSpeedsSupplier;

//         pivotController = new ProfiledPIDController(kP, kI, kD,
//                 new TrapezoidProfile.Constraints(kMaxVel, kMaxAcc));
//         rightController = new ProfiledPIDController(kP, kI, kD,
//                 new TrapezoidProfile.Constraints(kMaxVel, kMaxAcc));

//         pivotController.setTolerance(0.01); // meters
//         rightController.setTolerance(0.01);

//         deploypivotTrigger = new Trigger(this::travelingpivot).and(() -> isDeployed);
//         deployRightTrigger = new Trigger(this::travelingRight).and(() -> isDeployed);

//         deploypivotTrigger.onTrue(deploypivot());
//         deployRightTrigger.onTrue(deployRight());
//     }

//     private boolean travelingpivot() {
//         return chassisSpeedsSupplier.get().vyMetersPerSecond >
// MIN_SWITCH_ROBOT_VELOCITY.in(Meters);
//     }

//     private boolean travelingRight() {
//         return chassisSpeedsSupplier.get().vyMetersPerSecond <
// -MIN_SWITCH_ROBOT_VELOCITY.in(Meters);
//     }

//     public Command deploy() {
//         return this.runOnce(() -> isDeployed = true)
//                 .andThen(deploypivot().onlyIf(() -> !travelingpivot() && !travelingRight()))
//                 .withName("Deploy intake");
//     }

//     private Command deploypivot() {
//         return this.runOnce(() -> {
//             pivotController.setGoal(DEPLOY_POS.in(Meters));
//             rightController.setGoal(STOW_POS.in(Meters));

//             pivotIO.setSpinOutput(SPIN_VOLTAGE);
//             rightIO.stopSpin();
//         }).withName("Deploy pivot Intake");
//     }

//     private Command deployRight() {
//         return this.runOnce(() -> {
//             pivotController.setGoal(STOW_POS.in(Meters));
//             rightController.setGoal(DEPLOY_POS.in(Meters));

//             pivotIO.stopSpin();
//             rightIO.setSpinOutput(SPIN_VOLTAGE);
//         }).withName("Deploy Right Intake");
//     }

//     public Command stow() {
//         return this.runOnce(() -> {
//             isDeployed = false;
//             pivotController.setGoal(STOW_POS.in(Meters));
//             rightController.setGoal(STOW_POS.in(Meters));

//             pivotIO.stopSpin();
//             rightIO.stopSpin();
//         }).withName("Stow intakes");
//     }

//     @Override
//     public void periodic() {
//         // Update hardware inputs
//         pivotIO.updateInputs(pivotInputs);
//         rightIO.updateInputs(rightInputs);

//         // PID calculations
//         double pivotOutput = pivotController.calculate(pivotInputs.rackPosition.in(Meters));
//         double rightOutput = rightController.calculate(rightInputs.rackPosition.in(Meters));

//         pivotIO.setRackOutput(Volts.of(pivotOutput));
//         rightIO.setRackOutput(Volts.of(rightOutput));
//     }
// }
