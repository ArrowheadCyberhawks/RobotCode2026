package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem.HopperState;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterState;

public class ShootCommand extends Command {
    private final ShooterSubsystem shooter;
    private final HopperSubsystem hopper;
    private final BooleanSupplier inTrench;
    private final BooleanSupplier inTower;
    private final BooleanSupplier inPass;
    private boolean hopperTriggered = false;

    /**
     * @param inTrench supplier that returns true when the robot is inside the
     *                 trench zone. Hood position is handled externally by the
     *                 {@code inTrench.whileTrue(hood.down())} trigger.
     *                 This command only pauses the hopper and requests TRENCH state
     *                 on the shooter state machine.
     */
    public ShootCommand(ShooterSubsystem shooter, HopperSubsystem hopper, BooleanSupplier inTrench, BooleanSupplier inTower, BooleanSupplier inPass) {
        this.shooter = shooter;
        this.hopper = hopper;
        this.inTrench = inTrench;
        this.inTower = inTower;
        this.inPass = inPass;
        addRequirements(shooter, hopper);
        //RETEST THE TRENCH WITH THIS QUICK FIX IF IT  IS USED AT SOME POINT, MIGHT CAUSE ISSUES
        addRequirements(shooter, hopper, shooter.getHoodSubsystem(), shooter.getTurretSubsystem(), shooter.getFlywheelSubsystem());
    }

    @Override
    public void initialize() {
        shooter.requestState(ShooterState.AIM);
        hopperTriggered = false;
    }

    @Override
    public void execute() {
        if (inTrench.getAsBoolean()) {
            // Request TRENCH state so the shooter state machine knows we are inside the trench.
            // The hood going down is handled by the inTrench.whileTrue(hood.down()) trigger.
            // Pause the hopper and re-arm so it fires again once we leave the trench.
            shooter.requestState(ShooterState.TRENCH);
            hopper.setHopperState(HopperState.IDLE);
            hopperTriggered = false;
        } else if (inTower.getAsBoolean()) {
            shooter.requestState(ShooterState.IDLE);
            hopper.setHopperState(HopperState.IDLE);
        } else {
            // Normal AIM + fire sequence
            if (inPass.getAsBoolean()) {
                shooter.requestState(ShooterState.PASS);
            } else {
                shooter.requestState(ShooterState.AIM);
            }

            hopper.setHopperState(HopperState.KICKER);
            
            if (shooter.areAllSubsystemsAtGoal()) {
                hopperTriggered = true;
                hopper.setHopperState(HopperState.ON);
            }

            if (hopperTriggered) {
                hopper.setHopperState(HopperState.ON);
            }

            if (shooter.isFlipping()) {
                hopper.setHopperState(HopperState.IDLE);
            }

        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hopper.setHopperState(HopperState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
