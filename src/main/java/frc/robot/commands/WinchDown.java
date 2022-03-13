// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class WinchDown extends CommandBase {
  private Climber climber;
  /** Creates a new WinchDown. */
  public WinchDown(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!climber.getLeftMagnetSensorValue()) { 
      climber.resetLeftEncoders();        // true when see black tape
      climber.setLeft(0);
    }
    else {
      climber.setLeft(-1);
    }

    if (!climber.getRightMagnetSensorValue()) { // true when see black tape
      climber.resetRightEncoders();
      climber.setRight(0);
    }
    else {
      climber.setRight(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setWinch(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
