// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

/** Add your docs here. */
public enum ShooterPosition {
    FENDER_LOW(0, "Fender Low", 0.01, 0.01, 5000, 14000), 
    FENDER_HIGH(1, "Fender High", 0.01, 0.01, 5000, 16000),
    LINE(2, "Line", 0.0075, 0.01, 9500, 11000), 
    TARMAC(3, "Tarmac", 0.01, 0.01, 7500, 15000), 
    LAUNCHPAD(4, "Launchpad", 0.01, 0.01, 11000, 11500),
    INVALID(5, "Invalid", 0, 0, 0, 0);

    private int index;
    private String name;

    private Gains bottomGains;
    private Gains topGains;

    private double bottomSetpoint;
    private double topSetpoint;
    
    private ShooterPosition(int index, String name, double bottomPGain, double topPGain, double bottomSetpoint, double topSetpoint) {
      this.index = index;
      this.name = name;

      this.bottomSetpoint = bottomSetpoint;
      this.topSetpoint = topSetpoint;

      bottomGains = new Gains(bottomPGain, 0, 0, ((bottomSetpoint / 20800) * 1023.0 / bottomSetpoint));
      topGains = new Gains(topPGain, 0, 0, ((topSetpoint / 20800) * 1023.0 / topSetpoint));
    }

    public int getIndex() {
      return index;
    }

    public String getName() {
      return name;
    }

    public Gains getBottomGains() {
      return bottomGains;
    }

    public Gains getTopGains() {
      return topGains;
    }

    public double getBottomSetpoint(){
      return bottomSetpoint;
    }

    public double getTopSetpoint() {
      return topSetpoint;
    }

    public static ShooterPosition fromInt(int value) {
      for(ShooterPosition position : values()) {
        if(position.index == value) {
          return position;
        }
      }
      return INVALID;
    }

    public static int getHighestIndex() {
      int highestIndex = 0;
      for(ShooterPosition position : values()) {
        if(position.getIndex() > highestIndex) {
          highestIndex = position.index;
        }
      }
      return highestIndex - 1;
    }
}
