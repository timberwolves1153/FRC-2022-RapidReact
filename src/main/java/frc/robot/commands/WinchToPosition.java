// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class WinchToPosition extends CommandBase {
  Climber climb;
  int position;
  /** Creates a new WinchToPosition. */
  public WinchToPosition(int position, Climber climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = climb;
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.resetLeftEncoders();
    climb.resetRightEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.setWinchPosition(this.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
