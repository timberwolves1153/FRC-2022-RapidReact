// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;

public class DefaultLauncher extends CommandBase {
  private DoubleSupplier leftYStick;
  private Launcher launcher;

  /** Creates a new DefaultShooter. */
  public DefaultLauncher(DoubleSupplier leftYStick, Launcher launcher) {
    this.leftYStick = leftYStick;
    this.launcher = launcher;
    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(leftYStick.getAsDouble() < -0.5) {
      launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB);
      SmartDashboard.putString("Launcher Position", "Upper Hub");
    }
    if(leftYStick.getAsDouble() > 0.5) {
      launcher.setGainPreset(Launcher.ShooterPosition.LOWER_HUB);
      SmartDashboard.putString("Launcher Position", "Lower Hub");
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
