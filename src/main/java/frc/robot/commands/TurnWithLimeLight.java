/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnWithLimeLight extends PIDCommand {
  private boolean canFinishCommand = false;
  private int counter = 1;

  private Limelight limelight;
  private Drive drive;
  /**
   * Creates a new TurnWithLimelight.
   */
  public TurnWithLimeLight(Drive drive, Limelight limeLight) {
    super(
        // The controller that the command will use
        new Limelight().getController(),
        // This should return the measurement
        limeLight::getTargetX,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          System.out.println(output);
          if (limeLight.getTargetValues() == null) {
            drive.arcadeDrive(0, 0.5);
          } else {
            drive.arcadeDrive(-0.5, -1.1 * output);
          }
        }, 
        drive,
        limeLight);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(5, 5);
    getController().enableContinuousInput(-29.8, 29.8);
    this.limelight = limeLight;
    this.drive = drive;
  }
  @Override
  public void initialize() {
    super.initialize();
    //vision.setLedMode(3);
    //vision.setPipeline(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(counter % 20 == 0) {
      canFinishCommand = true;
      System.out.println(canFinishCommand);
    } else {
      canFinishCommand = false;
    }
    counter++;
    //return canFinishCommand && getController().atSetpoint();
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    //vision.setLedMode(1);
    //vision.setPipeline(0);
    drive.arcadeDrive(0, 0);
  }
}