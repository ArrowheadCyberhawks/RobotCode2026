package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootCommand extends Command {
    private final ShooterSubsystem shooter;
    private final HopperSubsystem hopper;
    private final BooleanSupplier inTrench;
    private boolean hopperTriggered = false;

    /**
     * @param inTrench supplier that returns true when the robot is inside the
     *                 trench zone. Hood position is handled externally by the
     *                 {@code inTrench.whileTrue(hood.down())} trigger.
     *                 This command only pauses the hopper and requests TRENCH state
     *                 on the shooter state machine.
     */
    public ShootCommand(ShooterSubsystem shooter, HopperSubsystem hopper, BooleanSupplier inTrench) {
        this.shooter = shooter;
        this.hopper = hopper;
        this.inTrench = inTrench;
        addRequirements(shooter, hopper); // hood is NOT required here — handled by external trigger
    }

    @Override
    public void initialize() {
        shooter.requestState(ShooterSubsystem.ShooterState.AIM);
        hopperTriggered = false;
    }

    @Override
    public void execute() {
        if (inTrench.getAsBoolean()) {
            // Request TRENCH state so the shooter state machine knows we are inside the trench.
            // The hood going down is handled by the inTrench.whileTrue(hood.down()) trigger.
            // Pause the hopper and re-arm so it fires again once we leave the trench.
            shooter.requestState(ShooterSubsystem.ShooterState.TRENCH);
            hopper.setHopperState(HopperSubsystem.HopperState.IDLE);
            hopperTriggered = false;
        } else {
            // Normal AIM + fire sequence
            shooter.requestState(ShooterSubsystem.ShooterState.AIM);
            if (!hopperTriggered && shooter.areAllSubsystemsAtGoal()) {
                hopper.setHopperState(HopperSubsystem.HopperState.ON);
                hopperTriggered = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hopper.setHopperState(HopperSubsystem.HopperState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
