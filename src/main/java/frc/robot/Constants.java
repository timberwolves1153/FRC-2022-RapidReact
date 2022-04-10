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
    // public static final double ksVolts = 0.61565;
    // public static final double kvVoltSecondsPerMeter = 2.5557;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.25684;

    // Final Constants
    // public static final double ksVolts = 0.58668;
    // public static final double kvVoltSecondsPerMeter = 2.5632;
    // public static final double kaVoltSecondsSquaredPerMeter = 0.31239;

    // //public static final double ksVolts = 0.6499;
     public static final double ksVolts = 0.86221;
     
    // //public static final double kvVoltSecondsPerMeter = 2.5338;
     public static final double kvVoltSecondsPerMeter = 2.605;
     
    public static final double kaVoltSecondsSquaredPerMeter = 0.76326;
    //  public static final double kaVoltSecondsSquaredPerMeter = 0.23929;
    


    public static final double kPDriveVel = -1.0001;
    public static final double kDDriveVel = 0.1;

    // public static final double kTrackwidthMeters = 0.694;
    //public static final double kTrackwidthMeters = 0.635;
    public static final double kTrackwidthMeters = 0.18644;
    public static final double kRobotLength = 0.96;
    public static final DifferentialDriveKinematics kDriveKinematics = 
        new DifferentialDriveKinematics(kTrackwidthMeters);
    
    public static final double kMaxSpeedMetersPerSecond = 2.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;

    public static final double kRamseteB = 2.0; // 0.10
    public static final double kRamseteZeta = 0.7;
    
    public static final double POSITION_TOLERANCE = 5.0;
    public static final double VELOCITY_TOLERANCE = 5.0;

    public static final double DT_GEAR_RATIO = (62.0 / 9.0) * (30.0 / 18.0);
    public static final double ENCODER_TICKS_PER_FALCON_REV = 2048.0;
    public static final double ENCODER_TICKS_PER_DT_WHEEL_REVOLUTION = ENCODER_TICKS_PER_FALCON_REV * DT_GEAR_RATIO;

    public static final double DT_WHEEL_DIAMETER = 6.0 / 39.37;
}
