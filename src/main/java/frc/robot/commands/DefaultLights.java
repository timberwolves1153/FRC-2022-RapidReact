// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.LEDLights;

public class DefaultLights extends CommandBase {
  private ColorSensor colorSensor;
  private LEDLights lights;

  /** Creates a new DefaultLights. */
  public DefaultLights(ColorSensor colorSensor, LEDLights lights) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.colorSensor = colorSensor;
    this.lights = lights;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(colorSensor.getDetectedBallColor().getName().equals("Blue")) {
      lights.setRGB(0, 0, 255);
    } else if(colorSensor.getDetectedBallColor().getName().equals("Red")) {
      lights.setRGB(255, 0, 0);
    } else {
      lights.setRGB(255, 20, 147);
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
