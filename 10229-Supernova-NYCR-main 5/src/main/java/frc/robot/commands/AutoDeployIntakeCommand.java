package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoDeployIntakeCommand extends Command {
    private final IntakeSubsystem intake;

    public AutoDeployIntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Optional: Any setup when the button is first pressed
        // System.out.println("COMMAND INITIALIZED");
    }

    @Override
    public void execute() {
        // Send arm to deployed postion
        intake.setHingePositionMM(IntakeConstants.kDeployPosition);
        intake.setRollerSpeed(IntakeConstants.kRollerIntakeSpeed);
        // System.out.println("COMMAND DO BE RUNNING");
    }

    @Override
    public boolean isFinished() {
        // Return false so it keeps running as long as the driver holds the
        // button/trigger
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // When the driver lets go, immediately stop the rollers for safety
        // The Default Command (Stow) will instantly take over after this ends
        intake.setRollerSpeed(IntakeConstants.kRollerStopSpeed);
    }

}
