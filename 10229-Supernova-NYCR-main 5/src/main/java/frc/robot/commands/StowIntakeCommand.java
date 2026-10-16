package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class StowIntakeCommand extends Command{
    private final IntakeSubsystem intake;

    public StowIntakeCommand(IntakeSubsystem intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        // Optional: Any setup when the command starts
    }

    @Override
    public void execute(){
        // Constantly tell the arm to hold the Stowed position (0.0)
        intake.setHingePositionMM(IntakeConstants.kStowPosition);

        // Roller Logic: Check if the arm is currently "traveling"
        // If the arm is further down then -0.02 (need to double check this) rotations, keep the hold speed
        if(intake.getHingeRotations() > 0.02){
            intake.setRollerSpeed(IntakeConstants.kRollerHoldSpeed);
        }
        // Once the arm reaches the top (greater than -0.02), shut the rollers off
        else {
            // System.out.println("stowing...");
            intake.setRollerSpeed(IntakeConstants.kRollerStopSpeed);
        }
    }

    @Override
    public boolean isFinished(){
        // Because this is the Default Command, it should NEVER finish
        // It should constnatly guard the stowed position and roller state
        return false;
    }

    @Override
    public void end(boolean interrupted){
        // Stop the rollers if a different command (like Deploy or Shoot) interrupts it
        intake.setRollerSpeed(IntakeConstants.kRollerStopSpeed);
    }

}
