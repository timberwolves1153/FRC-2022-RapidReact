// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ColorSensor extends SubsystemBase {
  public enum BallColor {
    BLUE("Blue"), RED("Red"), NONE("None");

    private String ballName;

    private BallColor(String name) {
      this.ballName = name;
    }

    public String getName() {
      return ballName;
    }
  }

  private NetworkTable colorSensorTable;

  private NetworkTableEntry redValue;
  private NetworkTableEntry greenValue;
  private NetworkTableEntry blueValue;
  private NetworkTableEntry irValue;

  private NetworkTableEntry proxValue;

  /** Creates a new ColorSensor. */
  public ColorSensor() {
    colorSensorTable = NetworkTableInstance.getDefault().getTable("colorsensor");

    redValue = colorSensorTable.getEntry("red");
    greenValue = colorSensorTable.getEntry("green");
    blueValue = colorSensorTable.getEntry("blue");
    irValue = colorSensorTable.getEntry("ir");

    proxValue = colorSensorTable.getEntry("proximity");

  }

  public double getRedValue() {
    return redValue.getDouble(0.0);
  }

  public double getGreenValue() {
    return greenValue.getDouble(0.0);
  }

  public double getBlueValue() {
    return blueValue.getDouble(0.0);
  }

  public double getIRValue() {
    return irValue.getDouble(0.0);
  }

  public double getProxValue() {
    return proxValue.getDouble(0.0);
  }

  public BallColor getDetectedBallColor() {
    if(getBlueValue() < 1700 && getRedValue() < 1700) {
      return BallColor.NONE;
    }
    if(getBlueValue() > getRedValue()) {
      return BallColor.BLUE;
    }
    if(getRedValue() > getBlueValue()) {
      return BallColor.RED;
    }
    else {
      return BallColor.NONE;
    }
  }

  public void updateShuffleboard() {
    SmartDashboard.putNumber("Color Sensor Red", getRedValue());
    SmartDashboard.putNumber("Color Sensor Green", getGreenValue());
    SmartDashboard.putNumber("Color Sensor Blue", getBlueValue());
    SmartDashboard.putNumber("Color Sensor IR", getIRValue());
    SmartDashboard.putNumber("Color Sensor Proximity", getProxValue());
    SmartDashboard.putString("Ball Detected", getDetectedBallColor().getName());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
