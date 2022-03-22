// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rumble extends SubsystemBase {
  private XboxController driveStick;
  private XboxController opStick;

  private boolean isRunning = false;

  /** Creates a new Rumble. */
  public Rumble(XboxController driveStick, XboxController opStick) {
    this.driveStick = driveStick;
    this.opStick = opStick;
  }

  public void oneBallPattern() {
    isRunning = true;
    driveStick.setRumble(RumbleType.kLeftRumble, 1);
    driveStick.setRumble(RumbleType.kRightRumble, 1);

    opStick.setRumble(RumbleType.kLeftRumble, 1);
    opStick.setRumble(RumbleType.kRightRumble, 1);
  } 

  @Override
  public void periodic() {
    //oneBallPattern();
  }
}
