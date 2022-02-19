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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimbForDistance;
import frc.robot.commands.DefaultCollect;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DefaultLauncher;
import frc.robot.commands.FullAutoCommandGroup;
import frc.robot.commands.PartialAutoCommandGroup;
import frc.robot.commands.TurnForDegrees;
import frc.robot.commands.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.ColorSensor;
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

  private JoystickButton opY;
  private JoystickButton opB;
  private JoystickButton opA;
  private JoystickButton opLeftBumper;
  private JoystickButton opRightBumper;

  private ClimbForDistance climbForDistance;
  
  //Instantiates all subsystems
  private Drive drive;
  private Collector collector;
  private Climber climber;
  private Launcher launcher;
  private ColorSensor colorSensor;

  private Trajectory exampleTrajectory;
  private Trajectory straightPathTrajectory;
  private Trajectory manualTrajectory1;
  private Trajectory manualTrajectory2;
  
  private RamseteCommand manualRamseteCommand1Full;
  private RamseteCommand manualRamseteCommand1Partial;
  private RamseteCommand manualRamseteCommand2;
  private RamseteCommand ramseteCommand;

  private SequentialCommandGroup fullAutoCommandGroup;
  private SequentialCommandGroup partialAutoCommandGroup;

  private String manualPath1 = "pathplanner/generatedJSON/ManualPath1.wpilib.json";
  private String manualPath2 = "pathplanner/generatedJSON/ManualPath2.wpilib.json";

  private SendableChooser<Command> autoCommandChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(){
    //declares the drive joystick as an XboxController in port 0 on the Driver Station
    driveStick = new XboxController(0);
    opStick = new XboxController(1);

    //declares all subsystems
    drive = new Drive();
    collector = new Collector();
    climber = new Climber();
    launcher = new Launcher();
    colorSensor = new ColorSensor();

    driveLeftBumper = new JoystickButton(driveStick, XboxController.Button.kLeftBumper.value);
    driveRightBumper = new JoystickButton(driveStick, XboxController.Button.kRightBumper.value);
    driveX = new JoystickButton(driveStick, XboxController.Button.kX.value);
    driveY = new JoystickButton(driveStick, XboxController.Button.kY.value);
    driveB = new JoystickButton(driveStick, XboxController.Button.kB.value);
    driveA = new JoystickButton(driveStick, XboxController.Button.kA.value);

    opY = new JoystickButton(opStick, XboxController.Button.kY.value);
    opB = new JoystickButton(opStick, XboxController.Button.kB.value);
    opA = new JoystickButton(opStick, XboxController.Button.kA.value);

    opLeftBumper = new JoystickButton(opStick, XboxController.Button.kLeftBumper.value);
    opRightBumper = new JoystickButton(opStick, XboxController.Button.kRightBumper.value);

    climbForDistance = new ClimbForDistance(5, climber);

    autoCommandChooser = new SendableChooser<Command>();
    
    drive.setDefaultCommand(new DefaultDrive(
      () -> driveStick.getLeftY(),
      () -> driveStick.getRightX(), 
      drive));

    launcher.setDefaultCommand(new DefaultLauncher(
      () -> opStick.getLeftY(),
      launcher));

    collector.setDefaultCommand(new DefaultCollect(
      () -> opStick.getLeftTriggerAxis(),
      () -> opStick.getRightTriggerAxis(), 
      collector));

    // Configure the buttons to start new commands when they are pressed or released
    
    generateTrajectories();

    fullAutoCommandGroup = new SequentialCommandGroup(
      new InstantCommand(() -> System.out.println("Running Full Auto")),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
      new InstantCommand(() -> launcher.setLauncherForPosition(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> launcher.feederOn(), launcher),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> collector.intake(), collector),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> launcher.feederOff(), launcher),
      new TurnForDegrees(165, drive),
      new InstantCommand(()-> drive.resetOdometry(manualTrajectory1.getInitialPose())),
      generateRamseteCommandFromTrajectory(manualTrajectory1),
      new TurnForDegrees(110, drive),
      new WaitCommand(0.25),
      new InstantCommand(()-> drive.resetOdometry(manualTrajectory2.getInitialPose())),
      generateRamseteCommandFromTrajectory(manualTrajectory2),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
      new InstantCommand(() -> launcher.setLauncherForPosition(), launcher),
      new InstantCommand(() -> launcher.feederOn(), launcher),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(4),
      new InstantCommand(() -> collector.stop(), collector),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.moverOff(), collector),
      new InstantCommand(() -> launcher.feederOff(), launcher),
      new InstantCommand(() -> collector.singulatorStop(), collector)
    );
    
    partialAutoCommandGroup = new SequentialCommandGroup(
      new InstantCommand(() -> System.out.println("Running Partial Auto")),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
      new InstantCommand(() -> launcher.setLauncherForPosition(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> launcher.feederOn(), launcher),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> collector.intake(), collector),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> launcher.feederOff(), launcher),
      new TurnForDegrees(165, drive),
      new InstantCommand(()-> drive.resetOdometry(manualTrajectory1.getInitialPose())),
      generateRamseteCommandFromTrajectory(manualTrajectory1),
      new TurnForDegrees(180, drive),
      new InstantCommand(()-> drive.resetOdometry(manualTrajectory1.getInitialPose())),
      generateRamseteCommandFromTrajectory(manualTrajectory1),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
      new InstantCommand(() -> launcher.setLauncherForPosition(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> launcher.feederOn(), launcher),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> collector.intake(), collector),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> launcher.feederOff(), launcher),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorStop(), collector)
    );

    autoCommandChooser.setDefaultOption("Partial Auto", partialAutoCommandGroup);
    autoCommandChooser.addOption("Full Auto", fullAutoCommandGroup);

    SmartDashboard.putData("Auto Command Chooser", autoCommandChooser);

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


    // driveB.whenPressed(new InstantCommand(() -> drive.resetEncoders(), drive));

    driveLeftBumper.whenPressed(new InstantCommand(() -> climber.winchDown()));
    driveLeftBumper.whenReleased(new InstantCommand(() -> climber.stop()));

    driveRightBumper.whenPressed(new InstantCommand(() -> climber.winchUp()));
    driveRightBumper.whenReleased(new InstantCommand(() -> climber.stop()));

    driveX.whenPressed(new InstantCommand(() -> climber.toggleSolenoid()));

    driveA.whenPressed(climbForDistance);
    driveA.whenReleased(() -> climbForDistance.cancel());

    opY.whenPressed(new InstantCommand(() -> launcher.setLauncherForPosition()));
    opY.whenReleased(new InstantCommand(() -> launcher.stop()));

    opB.whenPressed(new InstantCommand(() -> launcher.feederOn()));
    opB.whenReleased(new InstantCommand(() -> launcher.feederOff()));

    opA.whenPressed(new InstantCommand(() -> collector.toggleSolenoid()));

    opLeftBumper.whenPressed(new InstantCommand(() -> collector.intake()));
    opLeftBumper.whenReleased(new InstantCommand(() -> collector.stop()));

    opRightBumper.whenPressed(new InstantCommand(() -> collector.outake()));
    opRightBumper.whenReleased(new InstantCommand(() -> collector.stop()));
  }

  /**
   * Master method for updating the updateShuffleboard() method in each subsystem
   */
  public void updateShuffleboard() {
    drive.updateShuffleboard();
    launcher.updateShuffleboard();
    colorSensor.updateShuffleboard();
    climber.updateShuffleboard();
  }

  public void generateTrajectories(){
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters. Creates a list of Pose2d objects that the robot tries to replicate in real life.
    exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
          List.of(
            new Pose2d(0, 0, new Rotation2d()),
            new Pose2d(1, 0.5, new Rotation2d(Math.PI/4)),
            new Pose2d(2, 1.0, new Rotation2d(-Math.PI/4)),
            new Pose2d(3, 1.5, new Rotation2d(Math.PI*5/4)),
            new Pose2d(5, 2, new Rotation2d())
          ),
          // Pass config
          config);
    
    straightPathTrajectory = 
      TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(0, 0, new Rotation2d()),
          new Pose2d(2, 0, new Rotation2d())
        ), 
        config);

    // Creates a ramsete command, which combines all of the previous systems to create an autonomous motion profile that follows the trajectory we just created
    try {
      manualTrajectory1 = generateTrajectoryFromJSON(manualPath1);
      manualTrajectory2 = generateTrajectoryFromJSON(manualPath2);
    } catch (IOException e) {
      System.out.println("Could not read trajectory file.");
    }
    manualRamseteCommand1Full = generateRamseteCommandFromTrajectory(manualTrajectory1);
    manualRamseteCommand1Partial = generateRamseteCommandFromTrajectory(manualTrajectory1);
    manualRamseteCommand2 = generateRamseteCommandFromTrajectory(manualTrajectory2);

    ramseteCommand = generateRamseteCommandFromTrajectory(straightPathTrajectory);
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
  public Command getAutonomousCommand(){
    // Run path following command, then stop at the end.
  //return manualRamseteCommand2.andThen(() -> drive.tankDriveVolts(0, 0));
    return autoCommandChooser.getSelected();
  }
}
