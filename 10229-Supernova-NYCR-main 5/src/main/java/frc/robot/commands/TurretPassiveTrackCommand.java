package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretPassiveTrackCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final DoubleSupplier joystickInput;
    private final BooleanSupplier resetOverrideBtn; 
    private boolean manualOverride = false;
    private boolean inManual = false;

    public TurretPassiveTrackCommand(CommandSwerveDrivetrain dt, TurretSubsystem t, ShooterSubsystem s, DoubleSupplier joystickInput, BooleanSupplier resetOverrideBtn) {
        this.drivetrain = dt;
        this.turret = t;
        this.shooter = s;
        this.joystickInput = joystickInput;
        this.resetOverrideBtn = resetOverrideBtn;
        // ONLY requires Turret so we can drive/shoot simultaneously!
        addRequirements(turret); 
    }

    @Override
    public void initialize(){
        SmartDashboard.putBoolean("Turret Manual", inManual);
    }

    @Override
    public void execute() {

        SmartDashboard.putBoolean("Turret Manual", inManual);
        double rawSpeed = joystickInput.getAsDouble();
        double adjustedSpeed = MathUtil.applyDeadband(rawSpeed, 0.2);

        // *** MANUAL CONTROL ***
        if (adjustedSpeed != 0.0){
            manualOverride = true;
            turret.rotateTurret(adjustedSpeed * 0.5);
            inManual = true;
            return; // Exit early, ignore auto logic
        }

        if (manualOverride && !resetOverrideBtn.getAsBoolean()){
            turret.rotateTurret(0);
            return; // Hold manual position
        }

        if (resetOverrideBtn.getAsBoolean()){
            inManual = false;
            manualOverride = false;
        }
        
        Pose2d pose = drivetrain.getState().Pose;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        
        // Are we on the offensive half of the field?
        boolean isOffensive = (alliance == Alliance.Blue) 
            ? pose.getX() < ShooterConstants.BLUE_CENTERLINE_X 
            : pose.getX() > ShooterConstants.RED_CENTERLINE_X;

        if (isOffensive) {
            // ==========================================
            // OFFENSE: Aim at the Scoring Hub
            // ==========================================
            Translation2d hub = (alliance == Alliance.Blue) ? ShooterConstants.BLUE_HUB : ShooterConstants.RED_HUB;
            // System.out.println("hub: "+hub);
            double dist = pose.getTranslation().getDistance(hub);
            turret.trackLocationDynamic(hub, shooter.getTimeOfFlight(dist));
            
        } else {
            // ==========================================
            // DEFENSE: Aim at the Passing Zones
            // ==========================================
            double y_coord = pose.getY();
            Translation2d passZone; // Declare the variable, but don't calculate distance yet!
            
            // 1. Determine which specific pass zone to use based on Y coordinate
            if (alliance == Alliance.Blue) {
                passZone = (y_coord > ShooterConstants.CENTERLINE_Y) 
                    ? ShooterConstants.BLUE_PASS_ZONE_LEFT 
                    : ShooterConstants.BLUE_PASS_ZONE_RIGHT;
            } else {
                passZone = (y_coord > ShooterConstants.CENTERLINE_Y) 
                    ? ShooterConstants.RED_PASS_ZONE_RIGHT 
                    : ShooterConstants.RED_PASS_ZONE_LEFT;
            }
            
            // System.out.println("passZone: "+passZone);
            // 2. NOW calculate the distance to the correctly chosen pass zone
            double dist = pose.getTranslation().getDistance(passZone);
            
            // 3. Aim the turret and calculate Shoot-on-the-Move
            turret.trackLocationDynamic(passZone, shooter.getTimeOfFlight(dist)); 
        }
    }

    @Override
    public boolean isFinished() {
        // This is a default command, it should run forever to keep the turret locked on
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}