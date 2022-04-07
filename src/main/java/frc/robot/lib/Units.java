// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import frc.robot.Constants;

/** Add your docs here. */
public class Units {
    public static double falconTicksToRotations(double ticks) {
        return ticks / Constants.ENCODER_TICKS_PER_DT_WHEEL_REVOLUTION;
    }

    public static double falconRotationsToMeters(double rotations) {
        return rotations * (Constants.DT_WHEEL_DIAMETER * Math.PI);
    }
}
