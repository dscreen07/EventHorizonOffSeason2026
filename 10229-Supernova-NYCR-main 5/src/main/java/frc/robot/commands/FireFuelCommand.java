package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VortexSubsystem;
import frc.robot.subsystems.WormholeSubsystem;

public class FireFuelCommand extends Command {
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final WormholeSubsystem wormhole;
    private final VortexSubsystem vortex;
    
    private final Timer feedTimer = new Timer();

    public FireFuelCommand(ShooterSubsystem s, TurretSubsystem t, WormholeSubsystem w, VortexSubsystem v) {
        this.shooter = s;
        this.turret = t;
        this.wormhole = w;
        this.vortex = v;
        addRequirements(wormhole, vortex); // ONLY requires feeders!
    }

    @Override
    public void initialize() {
        feedTimer.restart();
    }

    @Override
    public void execute() {
        // State Lock Check: Are the muscles where the brain told them to be?
        // System.out.println("shooter.isReady(): "+shooter.isReady());
        // System.out.println("turret.isAimed(): "+turret.isAimed());
        if (shooter.isReady() && turret.isAimed()) {
            
            feedTimer.start();
            wormhole.runWormhole(); // Bottom feeder runs instantly

            if (feedTimer.hasElapsed(0.3)) {
                vortex.runVortex(); // Top feeder delays slightly for spacing
            } else {
                vortex.stopVortex();
            }
            
        } else {
            // Failsafe: Turret bumped or RPM dropped
            wormhole.stopWormhole();
            vortex.stopVortex();
            feedTimer.stop();
            //feedTimer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        wormhole.stopWormhole();
        vortex.stopVortex();
        feedTimer.stop();
    }
}