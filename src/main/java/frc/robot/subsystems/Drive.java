// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Units;

public class Drive extends SubsystemBase {
  private WPI_TalonFX rightMaster;
  private WPI_TalonFX rightFollower;
  private WPI_TalonFX leftMaster;
  private WPI_TalonFX leftFollower;

  private TalonFXSensorCollection rightEncoder;
  private TalonFXSensorCollection leftEncoder;

  private DifferentialDrive differentialDrive;

  private ADIS16470_IMU imu;
  
  private DifferentialDriveOdometry odometry;
  
  private Pose2d pose;

  private Field2d field2d;

  private NeutralMode currentMode;

  private double driveSide = 1;

  /** Creates a new Drive. */
  public Drive() {
    leftMaster = new WPI_TalonFX(0);
    leftFollower = new WPI_TalonFX(1);
    rightMaster = new WPI_TalonFX(2);
    rightFollower = new WPI_TalonFX(3);

    rightFollower.follow(rightMaster);
    leftFollower.follow(leftMaster);

    rightEncoder = new TalonFXSensorCollection(rightMaster);
    leftEncoder = new TalonFXSensorCollection(leftMaster);

    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);

    currentMode = NeutralMode.Brake;

    rightMaster.setInverted(true);
    rightFollower.setInverted(true);

    differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

    pose = new Pose2d();

    imu = new ADIS16470_IMU();

    odometry = new DifferentialDriveOdometry(new Rotation2d(imu.getAngle()));

    field2d = new Field2d();

    SmartDashboard.putData("Field", field2d);
  }

  /**
   * Resets the timer used in the DifferentialDrive safety object to prevent MotorSafety errors
   */
  public void feed() {
    differentialDrive.feed();
  }

  /**
   * Creates a drive signal with speed and rotation percentages
   * @param speed forward/backward speed percentage
   * @param rotation rotation speed percentage
   */
  public void arcadeDrive(double speed, double rotation) {
    differentialDrive.arcadeDrive(driveSide * speed, -rotation);
  }

  /**
   * Provides individual voltages to motor controller groups
   * @param leftVolts Left motor controller group volts
   * @param rightVolts Right motor controller group volts
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(-leftVolts);
    rightMaster.setVoltage(-rightVolts);
    feed();
  }

  public void turnWithLimelight(Limelight limelight) {
    if(limelight.targetExists()) {
      arcadeDrive(0, -limelight.getController().calculate(limelight.getTargetX(), limelight.getController().getSetpoint()));
    } else {
      arcadeDrive(0, 0.5);
    }
  }

  /**
   * Resets the robot odometry to a new initial pose and a gyro heading of 0 degrees. Also reset encoders.
   * @param pose desired odometry starting Pose2d
   */
  public void resetOdometry(Pose2d pose) {
    resetImu();
    resetEncoders();
    odometry.resetPosition(pose, new Rotation2d(getHeading()));
  }

  /**
   * Sets the left and right encoder positions to 0
   */
  public void resetEncoders() {
    leftEncoder.setIntegratedSensorPosition(0, 0);
    rightEncoder.setIntegratedSensorPosition(0, 0);
  }

  /**
   * Sets the gyro heading to 0 degrees.
   */
  public void resetImu() {
    imu.reset();
  }

  public void toggleDriveSide() {
    driveSide = -driveSide;
  }

  /**
   * Gets the speeds of the wheels in meters per second using a conversion factor
   * @return DifferentialDriveWheelSpeeds object containing the speed of each robot side in meters per second
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      Units.falconRotationsToMeters(Units.falconTicksToRotations(getLeftEncoderVelocity())), 
      Units.falconRotationsToMeters(Units.falconTicksToRotations(getRightEncoderVelocity()))
    );
  }

  /**
   * Gets the gyro heading in degrees
   * @return gyro heading in degrees
   */
  public double getHeading() {
    return imu.getAngle();
  }

  /**
   * Gets the robot's current pose
   * @return current robot Pose2d
   */
  public Pose2d getPose() {
    return pose;
  }

  public double getRightSideDistanceMeters() {
    return Units.falconRotationsToMeters(Units.falconTicksToRotations(getRightEncoderPosition()));
  }

  public double getLeftSideDistanceMeters() {
    return Units.falconRotationsToMeters(Units.falconTicksToRotations(getLeftEncoderPosition()));
  }

  /**
   * Gets the right encoder cumulative position in encoder ticks
   * @return right encoder cumulative position in encoder ticks
   */
  public double getRightEncoderPosition() {
    return rightEncoder.getIntegratedSensorPosition();
  }

  /**
   * Gets the left encoder cumulative position in encoder ticks
   * @return left encoder cumulative position in encoder ticks
   */
  public double getLeftEncoderPosition() {
    return -leftEncoder.getIntegratedSensorPosition();
  }

  /**
   * Gets the right encoder velocity in encoder ticks per second 
   * @return right encoder velocity in encoder ticks per second
   */
  public double getRightEncoderVelocity() {
    return rightEncoder.getIntegratedSensorVelocity() * 10;
  }

  /**
   * Gets the left encoder velocity in encoder ticks per second
   * @return Left encoder velocity in encoder ticks per second
   */
  public double getLeftEncoderVelocity() {
    return -leftEncoder.getIntegratedSensorVelocity() * 10;
  }

  public void setCoast() {
    rightMaster.setNeutralMode(NeutralMode.Coast);
    rightFollower.setNeutralMode(NeutralMode.Coast);
    leftMaster.setNeutralMode(NeutralMode.Coast);
    leftFollower.setNeutralMode(NeutralMode.Coast);

    currentMode = NeutralMode.Coast;
  }

  public void setBrake() {
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightFollower.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftFollower.setNeutralMode(NeutralMode.Brake);

    currentMode = NeutralMode.Brake;
  }

  public NeutralMode getCurrentNeutralMode() {
    return currentMode;
  }

  /**
   * Contains all Shuffleboard print statements
   */
  public void updateShuffleboard() {
    // SmartDashboard.putNumber("Left Encoder Position", getLeftEncoderPosition());
    // SmartDashboard.putNumber("Right Encoder Position", getRightEncoderPosition());
    // SmartDashboard.putNumber("Left t/s", getLeftEncoderVelocity());
    // SmartDashboard.putNumber("Right t/s", getRightEncoderVelocity());
    // SmartDashboard.putNumber("Left m/s", Units.falconRotationsToMeters(Units.falconTicksToRotations(getLeftEncoderVelocity())));
    // SmartDashboard.putNumber("Right m/s", Units.falconRotationsToMeters(Units.falconTicksToRotations(getRightEncoderVelocity())));
    // SmartDashboard.putNumber("Left Encoder Distance", Units.falconRotationsToMeters(Units.falconTicksToRotations(getLeftEncoderPosition())));
    // SmartDashboard.putNumber("Right Encoder Distance", Units.falconRotationsToMeters(Units.falconTicksToRotations(getRightEncoderPosition())));
    
    // SmartDashboard.putNumber("Gyro Heading Z", imu.getAngle());
    // SmartDashboard.putNumber("Gyro Complementary X", imu.getXComplementaryAngle());
    // SmartDashboard.putNumber("Gryo Complementary Y", imu.getYComplementaryAngle());

    SmartDashboard.putNumber("Odometry X", pose.getX());
    SmartDashboard.putNumber("Odometry Y", pose.getY());
    SmartDashboard.putNumber("Odometry Rotation", pose.getRotation().getDegrees());
  }

  public void setFieldTrajectory(String pathName, Trajectory traj) {
    field2d.getObject(pathName).setTrajectory(traj);
  }

  @Override
  public void periodic() {
    pose = odometry.update(Rotation2d.fromDegrees(imu.getAngle()), 
    Units.falconRotationsToMeters(Units.falconTicksToRotations(getLeftEncoderPosition())), 
    Units.falconRotationsToMeters(Units.falconTicksToRotations(getRightEncoderPosition())));

    field2d.setRobotPose(pose);
  }
}
