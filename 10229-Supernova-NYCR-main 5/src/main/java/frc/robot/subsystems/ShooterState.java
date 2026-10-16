// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;

/** Add your docs here. */
public class ShooterState implements Interpolatable<ShooterState>{

    public final double rpm;
    public final double hoodAngle;
    public final double timeOfFlight;

    public ShooterState (double rpm, double hoodAngle, double timeOfFlight){
        this.rpm = rpm;
        this.hoodAngle = hoodAngle;
        this.timeOfFlight = timeOfFlight;
    }

    @Override
    public ShooterState interpolate(ShooterState endValue, double t){
        double interpolatedRpm = MathUtil.interpolate(this.rpm, endValue.rpm, t);
        double interpolatedHood = MathUtil.interpolate(this.hoodAngle, endValue.hoodAngle, t);
        double interpolatedToF = MathUtil.interpolate(this.timeOfFlight, endValue.timeOfFlight, t);
        return new ShooterState(interpolatedRpm, interpolatedHood, interpolatedToF);
    }

    public static final Interpolator<ShooterState> interpolater = (startValue, endValue, t) -> {
        return startValue.interpolate(endValue, t);
    };
}
