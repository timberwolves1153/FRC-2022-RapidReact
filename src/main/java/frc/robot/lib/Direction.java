// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

/** Add your docs here. */
public enum Direction {
    kForwards(1), kBackwards(-1);

    private int direction;

    private Direction(int direction) {
      this.direction = direction;
    }

    public int getDirection() {
      return direction;
    }
}
