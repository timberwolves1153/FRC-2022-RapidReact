// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Launcher.Direction;
import frc.robot.subsystems.Launcher.ShooterPosition;


public class DefaultLauncher extends CommandBase {
  private DoubleSupplier leftYStick;
  private Launcher launcher;
  private boolean canSwitch = true;

  /** Creates a new DefaultShooter. */
  public DefaultLauncher(DoubleSupplier leftYStick, DoubleSupplier rightYStick, Launcher launcher) {
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
      if(canSwitch) {
        launcher.cycleGainPreset(Direction.kForwards);
        canSwitch = false;
        if (launcher.getSelectedPosition() == ShooterPosition.TARMAC_HIGH && leftYStick.getAsDouble() < -0.5){
          
        }
      }
    }
    else if(leftYStick.getAsDouble() > 0.5) {
      if(canSwitch) {
        launcher.cycleGainPreset(Direction.kBackwards);
        canSwitch = false;
      }
    }
    else {
      canSwitch = true;
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
