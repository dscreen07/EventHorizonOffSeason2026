package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VortexConstants;

public class VortexSubsystem extends SubsystemBase{

    private final TalonFX vortex = new TalonFX(VortexConstants.VORTEX_ID, "Colby");
    private final DutyCycleOut vortexRequest = new DutyCycleOut(0.0);

    // Anti-jam State Variables
    private boolean isUnjamming = false;
    private final Timer unjamTimer = new Timer();

    public VortexSubsystem(){
        configureHardware();
    }

    public void runVortex() {
        // Check if we are curretly in the middle of an unjam cycle
        if(isUnjamming){
            // If the timer is up, stop unjamming and reset
            if(unjamTimer.hasElapsed(VortexConstants.UNJAM_DURATION_SECONDS)){
                isUnjamming = false;
                unjamTimer.stop();
                unjamTimer.reset();
            } else {
                // Still unjamming: run in reverse and exit the method early
                vortex.setControl(vortexRequest.withOutput(VortexConstants.VORTEX_SPEED));
                return;
            }
        }

        // Normal operation: Check for a new jam
        // Phoenix 6 API to get Stator Current
        double currentAmps = vortex.getStatorCurrent().getValueAsDouble();

        if (currentAmps > VortexConstants.JAM_CURRENT_THRESHOLD){
            // Jam detected! Start the unjam process
            isUnjamming = true;
            unjamTimer.restart();   // Resets to 0 and starts counting
            vortex.setControl(vortexRequest.withOutput(VortexConstants.VORTEX_REVERSE_SPEED));
        } else {
            // No jam, run normally
            vortex.setControl(vortexRequest.withOutput(VortexConstants.VORTEX_SPEED));
        }
    }

    public void stopVortex() {
        // Reset state so it doesn't instantly run in reverse next time it starts 
        isUnjamming = false;
        unjamTimer.stop();
        unjamTimer.restart();
        vortex.setControl(vortexRequest.withOutput(0.0));
    }

    public boolean isUnjamming(){
        return isUnjamming;
    }

    public void configureHardware() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;


        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60.0;    // Max amps before hardware intervenes

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 25.0; // Max amps before we start throttling 40 before
        // config.CurrentLimits.SupplyCurrentLowerLimit = 30.0; // Drop to 30A if stalled
        // config.CurrentLimits.SupplyCurrentLowerTime = 0.5; // Wait half a second of stalling before dropping the limit

        vortex.getConfigurator().apply(config);
    }

}
