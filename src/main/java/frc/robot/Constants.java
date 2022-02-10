// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //public static final double ksVolts = 0.58612;
    public static final double ksVolts = 0.51344;
    //public static final double kvVoltSecondsPerMeter = 2.8094;
    public static final double kvVoltSecondsPerMeter = 2.8269;
    //public static final double kaVoltSecondsSquaredPerMeter = 0.11531;
    public static final double kaVoltSecondsSquaredPerMeter = 0.16096;

    public static final double kPDriveVel = 0.00001;
    //public static final double kPDriveVel = 2;
    public static final double kDDriveVel = 0.0005;
    //public static final double kDDriveVel = 0.0;

    public static final double kTrackwidthMeters = 0.694;
    public static final double kRobotLength = 0.79;
    public static final DifferentialDriveKinematics kDriveKinematics = 
        new DifferentialDriveKinematics(kTrackwidthMeters);
    
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
