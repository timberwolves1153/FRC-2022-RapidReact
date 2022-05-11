// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DefaultDrive extends CommandBase {
  private DoubleSupplier speed;
  private DoubleSupplier rotation;
  private DoubleSupplier rightTrigger;

  private Drive drive;

  /** Creates a new DefaultDrive. */
  public DefaultDrive(DoubleSupplier speed, DoubleSupplier rotation, DoubleSupplier rightTrigger, Drive drive) {
    this.speed = speed;
    this.rotation = rotation;
    this.rightTrigger = rightTrigger;
    this.drive = drive;
    

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(rightTrigger.getAsDouble() != 0) {
      drive.arcadeDrive(0 ,0);
    } else {
      drive.arcadeDrive(speed.getAsDouble(), 0.7 * rotation.getAsDouble());
    }
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
