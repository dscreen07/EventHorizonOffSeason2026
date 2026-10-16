package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WormholeConstants;

public class WormholeSubsystem extends SubsystemBase {
    private final TalonFX wormhole = new TalonFX(WormholeConstants.WORMHOLE_ID, "Colby");
    private final DutyCycleOut wormholeRequest = new DutyCycleOut(0.0);

    public WormholeSubsystem() {
        configureHardware();
    }

    public void runWormhole() {
        wormhole.setControl(wormholeRequest.withOutput(WormholeConstants.WORMHOLE_SPEED));
    }

    public void stopWormhole() {
        wormhole.setControl(wormholeRequest.withOutput(0.0));
    }

    public void configureHardware() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60.0;    // Max amps before hardware intervenes

        // Prevent battery brownouts
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 20.0; // Max amps before we start throttling 40 before
        
        wormhole.getConfigurator().apply(config);
    }
}
