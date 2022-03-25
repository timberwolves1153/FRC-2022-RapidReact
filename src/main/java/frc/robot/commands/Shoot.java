// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Launcher.ShooterPosition;

public class Shoot extends CommandBase {
  private Launcher launcher;
  private Limelight limelight;

  private ShooterPosition previousPosition = ShooterPosition.LOWER_HUB;

  /** Creates a new Shoot. */
  public Shoot(Launcher launcher, Limelight limelight) {
    this.launcher = launcher;
    this.limelight = limelight;

    addRequirements(launcher, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcher.pidOn();

    if(!launcher.getOverride()) {
      if(limelight.calcDistance() < 65) {
        launcher.setGainPreset(ShooterPosition.UPPER_HUB);
      } else if(limelight.calcDistance() > 65 && limelight.calcDistance() < 95) {
        launcher.setGainPreset(ShooterPosition.DEAD_ZONE);
      } else if(limelight.calcDistance() > 95) {
        launcher.setGainPreset(ShooterPosition.TARMAC_HIGH);
      }
    }
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.pidOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
