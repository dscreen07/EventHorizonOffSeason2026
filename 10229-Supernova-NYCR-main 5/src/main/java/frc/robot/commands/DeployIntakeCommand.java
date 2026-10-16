package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final BooleanSupplier outtakeButtonPressed;

    public DeployIntakeCommand(IntakeSubsystem intake, BooleanSupplier outtakeButtonPressed) {
        this.intake = intake;
        this.outtakeButtonPressed = outtakeButtonPressed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Optional: Any setup when the button is first pressed
    }

    @Override
    public void execute() {
        // Send arm to deployed postion
        intake.setHingePositionMM(IntakeConstants.kDeployPosition);

        // Check the B button to decide the roller direction
        if (outtakeButtonPressed.getAsBoolean()) {
            // If B is held, spit the fuel out
            intake.setRollerSpeed(IntakeConstants.kRollerOuttakeSpeed);
        } else {
            // Otherwise, pull the fuel in normally
            intake.setRollerSpeed(IntakeConstants.kRollerIntakeSpeed);
        }
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
