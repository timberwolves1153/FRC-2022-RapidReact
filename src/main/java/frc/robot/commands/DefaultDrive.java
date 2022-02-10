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

  private Drive drive;

  /** Creates a new DefaultDrive. */
  public DefaultDrive(DoubleSupplier speed, DoubleSupplier rotation, Drive drive) {
    this.speed = speed;
    this.rotation = rotation;
    this.drive = drive;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Drives the robot using the input taken from the joysticks; note the 0.8, which reduces the magnitude of the speed or
    //rotation to 80% of its original value to reduce the sensitivity of driving
    drive.arcadeDrive(0.8 * speed.getAsDouble(), 0.8 * rotation.getAsDouble());
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
