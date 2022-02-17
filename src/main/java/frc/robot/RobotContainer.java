// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimbForDistance;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.TurnForDegrees;
import frc.robot.commands.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //instantiates a new drive joystick with the XboxController class
  private XboxController driveStick;
  private XboxController opStick;

  private JoystickButton driveB;
  private JoystickButton driveLeftBumper;
  private JoystickButton driveRightBumper;
  private JoystickButton driveX;
  private JoystickButton driveY;
  private JoystickButton driveA;
  private ClimbForDistance climbForDistance;
  
  //Instantiates all subsystems
  private Drive drive;
  //private Collector collector;
  private Climber climber;
  //private Launcher launcher;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //declares the drive joystick as an XboxController in port 0 on the Driver Station
    driveStick = new XboxController(0);

    //declares all subsystems
    drive = new Drive();
    //collector = new Collector();
    climber = new Climber();
    //launcher = new Launcher();

    
    driveLeftBumper = new JoystickButton(driveStick, XboxController.Button.kLeftBumper.value);
    driveRightBumper = new JoystickButton(driveStick, XboxController.Button.kRightBumper.value);
    driveX = new JoystickButton(driveStick, XboxController.Button.kX.value);
    driveY = new JoystickButton(driveStick, XboxController.Button.kY.value);
    driveB = new JoystickButton(driveStick, XboxController.Button.kB.value);
    driveA = new JoystickButton(driveStick, XboxController.Button.kA.value);



    climbForDistance = new ClimbForDistance(5, climber);

    
    drive.setDefaultCommand(new DefaultDrive(
      () -> driveStick.getLeftY(), 
      () -> driveStick.getRightX(), 
      drive));
    // Configure the buttons to start new commands when they are pressed or released
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
   // driveLeftBumper.whenPressed(new InstantCommand(() -> collector.collectForward(), collector));
   // driveLeftBumper.whenReleased(new InstantCommand(() -> collector.stop(), collector));


    // driveRightBumper.whenPressed(new InstantCommand(() -> collector.collectReverse(), collector));
    // driveRightBumper.whenReleased(new InstantCommand(() -> collector.stop(), collector));


    // driveX.whenPressed(new InstantCommand(() -> collector.set(DoubleSolenoid.Value.kForward), collector));
    // driveY.whenPressed(new InstantCommand(() -> collector.set(DoubleSolenoid.Value.kReverse), collector));


    // driveB.whenPressed(new InstantCommand(() -> drive.resetEncoders(), drive));

    driveLeftBumper.whenPressed(new InstantCommand(() -> climber.winchDown()));
    driveLeftBumper.whenReleased(new InstantCommand(() -> climber.stop()));



    driveRightBumper.whenPressed(new InstantCommand(() -> climber.winchUp()));
    driveRightBumper.whenReleased(new InstantCommand(() -> climber.stop()));

    driveA.whenPressed(climbForDistance);
    driveA.whenReleased(() -> climbForDistance.cancel());


  }

  //Climber SmartDashboard
  public void updateDashboard() {
    SmartDashboard.putBoolean("rightWinch", climber.getRightMagnetSensorValue());
    SmartDashboard.putBoolean("leftWinch", climber.getLeftMagnetSensorValue());
    SmartDashboard.putNumber("rightEncoder", climber.getRightEncoder());
    SmartDashboard.putNumber("leftEncoder", climber.getLeftEncoder());
    SmartDashboard.putNumber("Accelerometer X", climber.getAccelerationX());
    SmartDashboard.putNumber("Accelerometer Y", climber.getAccelerationY());
    SmartDashboard.putNumber("Accelerometer Z", climber.getAccelerationZ());
   }



  /**
   * Master method for updating the updateShuffleboard() method in each subsystem
   */
  public void updateShuffleboard() {
    drive.updateShuffleboard();
    //launcher.updateShuffleboard();
  }

  public Trajectory generateTrajectoryFromJSON(String trajectoryPath) throws IOException{
    Path path = Filesystem.getDeployDirectory().toPath().resolve(trajectoryPath);
    Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(path);

    return trajectory;
  }

  public RamseteCommand generateRamseteCommandFromTrajectory(Trajectory trajectory) {
    return
    new RamseteCommand(
            trajectory,
            drive::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            drive::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
            new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
            // RamseteCommand passes volts to the callback
            (leftVolts, rightVolts) -> {
              //System.out.println("Left Volts   " + leftVolts + "               Right Volts:  " + rightVolts);
              SmartDashboard.putNumber("Left Volts ", leftVolts);
              SmartDashboard.putNumber("Right Volts ", rightVolts);
              drive.tankDriveVolts(leftVolts, rightVolts);
            },
            //drive::tankDriveVolts,
            drive);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() throws IOException {
    String manualPath1 = "pathplanner/generatedJSON/ManualPath1.wpilib.json";
    String manualPath2 = "pathplanner/generatedJSON/ManualPath2.wpilib.json";

    //DifferentialDriveVoltageConstraint is an object that uses a feedforward device, the DriveKinematics, and a max voltage of 10 to describe how the voltage should change based
    //on position error. The SimpleMotorFeedforward changes the voltage while the kinematics describes how the width of the robot affects the turning of the robot. The max voltage
    //of 10 ensures that the robot doesn't provide more than 10 volts to the motors to prevent brownouts.
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    // Create config for trajectory; describes the max acceleration and velocity while using the voltage constraint from before to regulate the voltage and providing the trajectory
    // with the DriveKinematics
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters. Creates a list of Pose2d objects that the robot tries to replicate in real life.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
          //Creates the list of poses that define the trajectory. Think of the poses as points on a graph that the robot will connect the dots with
          List.of(
            //Sets the initial pose, the one that the odometry is set to, to (0,0) with a rotation of 0. This tells the trajectory that the robot is at the "origin" 
            //of its graph facing the positive x direction
            new Pose2d(0, 0, new Rotation2d()),
            //Sets the next pose to be 2 meters in front of the original pose, causing the robot to move forwards to meters.
            new Pose2d(1, 0.5, new Rotation2d(Math.PI/4)),
            new Pose2d(2, 1.0, new Rotation2d(-Math.PI/4)),
            new Pose2d(3, 1.5, new Rotation2d(Math.PI*5/4)),
            new Pose2d(5, 2, new Rotation2d())
          ),
          // Pass config
          config);
    
    Trajectory straightPathTrajectory = 
      TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(0, 0, new Rotation2d()),
          new Pose2d(2, 0, new Rotation2d())
        ), 
        config);

    

    // Creates a ramsete command, which combines all of the previous systems to create an autonomous motion profile that follows the trajectory we just created
    Trajectory manualTrajectory1 = generateTrajectoryFromJSON(manualPath1);
    Trajectory manualTrajectory2 = generateTrajectoryFromJSON(manualPath2);

    RamseteCommand manualRamseteCommand1 = generateRamseteCommandFromTrajectory(manualTrajectory1);
    RamseteCommand manualRamseteCommand2 = generateRamseteCommandFromTrajectory(manualTrajectory2);


    RamseteCommand ramseteCommand = generateRamseteCommandFromTrajectory(straightPathTrajectory);

    // Reset odometry to the starting pose of the trajectory.
    drive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
  //return manualRamseteCommand2.andThen(() -> drive.tankDriveVolts(0, 0));
  return new SequentialCommandGroup(
    new InstantCommand(()-> drive.resetOdometry(manualTrajectory1.getInitialPose())),
    manualRamseteCommand1,
    new TurnForDegrees(110, drive),
    new WaitCommand(0.25),
    new InstantCommand(()-> drive.resetOdometry(manualTrajectory2.getInitialPose())),
    manualRamseteCommand2
  );
  }
}
