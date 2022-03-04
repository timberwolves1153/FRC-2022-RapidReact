// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
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
import frc.robot.commands.ClimbForDistance;
import frc.robot.commands.CollectForDistance;
import frc.robot.commands.DefaultCollect;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DefaultLauncher;
import frc.robot.commands.TurnForDegrees;
import frc.robot.commands.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.ColorSensor.BallColor;
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
  private JoystickButton driveStart;
  private JoystickButton driveBack;
  private JoystickButton driveLeftJoystickButton;
  private JoystickButton driveRightJoystickButton;

  private JoystickButton opY;
  private JoystickButton opB;
  private JoystickButton opA;
  private JoystickButton opLeftBumper;
  private JoystickButton opRightBumper;

  private ClimbForDistance climbForDistance;
  private CollectForDistance collectForDistance;
  
  //Instantiates all subsystems
  private Drive drive;
  private Collector collector;
  private Climber climber;
  private Launcher launcher;
  private ColorSensor colorSensor;
  private LEDLights ledLights;

  private Trajectory fourBallAutoTrajectory1;
  private Trajectory fourBallAutoTrajectory2;
  private Trajectory fourBallAutoTrajectory3;
  private Trajectory fourBallAutoTrajectory4;

  private Trajectory manualTrajectory1;
 
  private SequentialCommandGroup twoBallAutoCommandGroupRight;
  private SequentialCommandGroup twoBallAutoCommandGroupLeft;
  private SequentialCommandGroup fourBallAutoCommandGroup;

  private String manualPath1 = "pathplanner/generatedJSON/ManualPath1.wpilib.json";
  private String fourBallAutoPath1 = "pathplanner/generatedJSON/FourBallAutoPath1.wpilib.json";
  private String fourBallAutoPath2 = "pathplanner/generatedJSON/FourBallAutoPath2.wpilib.json";
  private String fourBallAutoPath3 = "pathplanner/generatedJSON/FourBallAutoPath3.wpilib.json";
  private String fourBallAutoPath4 = "pathplanner/generatedJSON/FourBallAutoPath4.wpilib.json";

  private SendableChooser<Command> autoCommandChooser;
  public SendableChooser<BallColor> allianceColor;

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
    ledLights = new LEDLights();

    driveLeftBumper = new JoystickButton(driveStick, XboxController.Button.kLeftBumper.value);
    driveRightBumper = new JoystickButton(driveStick, XboxController.Button.kRightBumper.value);
    driveX = new JoystickButton(driveStick, XboxController.Button.kX.value);
    driveY = new JoystickButton(driveStick, XboxController.Button.kY.value);
    driveB = new JoystickButton(driveStick, XboxController.Button.kB.value);
    driveA = new JoystickButton(driveStick, XboxController.Button.kA.value);
    driveStart = new JoystickButton(driveStick, XboxController.Button.kStart.value);
    driveBack = new JoystickButton(driveStick, XboxController.Button.kBack.value);
    driveLeftJoystickButton = new JoystickButton(driveStick, XboxController.Button.kLeftStick.value);
    driveRightJoystickButton = new JoystickButton(driveStick, XboxController.Button.kRightStick.value);

    opY = new JoystickButton(opStick, XboxController.Button.kY.value);
    opB = new JoystickButton(opStick, XboxController.Button.kB.value);
    opA = new JoystickButton(opStick, XboxController.Button.kA.value);

    opLeftBumper = new JoystickButton(opStick, XboxController.Button.kLeftBumper.value);
    opRightBumper = new JoystickButton(opStick, XboxController.Button.kRightBumper.value);

    climbForDistance = new ClimbForDistance(5, climber);
    collectForDistance = new CollectForDistance(5, collector);

    autoCommandChooser = new SendableChooser<Command>();
    allianceColor = new SendableChooser<BallColor>();
    
    drive.setDefaultCommand(new DefaultDrive(
      () -> driveStick.getLeftY(),
      () -> driveStick.getRightX(), 
      drive));

    launcher.setDefaultCommand(new DefaultLauncher(
      () -> opStick.getLeftY(),
      () -> opStick.getRightY(),
      launcher));

    collector.setDefaultCommand(new DefaultCollect(
      () -> opStick.getLeftTriggerAxis(),
      () -> opStick.getRightTriggerAxis(), 
      collector));

    // Configure the buttons to start new commands when they are pressed or released
    
    generateTrajectories();
    
    twoBallAutoCommandGroupRight = new SequentialCommandGroup(
      new InstantCommand(() -> System.out.println("Running Partial Auto")),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
      new InstantCommand(() -> launcher.setLauncherForPosition(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new TurnForDegrees(155, drive),
      new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
      new InstantCommand(() -> collector.intake(), collector),
      new InstantCommand(()-> drive.resetOdometry(manualTrajectory1.getInitialPose())),
      generateRamseteCommandFromTrajectory(manualTrajectory1),
      new TurnForDegrees(180, drive),
      new InstantCommand(()-> drive.resetOdometry(manualTrajectory1.getInitialPose())),
      generateRamseteCommandFromTrajectory(manualTrajectory1),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
      new InstantCommand(() -> launcher.setLauncherForPosition(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> collector.intake(), collector),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorStop(), collector)
    );

    twoBallAutoCommandGroupLeft = new SequentialCommandGroup(
      new InstantCommand(() -> System.out.println("Running Partial Auto")),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
      new InstantCommand(() -> launcher.setLauncherForPosition(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new TurnForDegrees(195, drive),
      new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
      new InstantCommand(() -> collector.intake(), collector),
      new InstantCommand(()-> drive.resetOdometry(manualTrajectory1.getInitialPose())),
      generateRamseteCommandFromTrajectory(manualTrajectory1),
      new TurnForDegrees(165, drive),
      new InstantCommand(()-> drive.resetOdometry(manualTrajectory1.getInitialPose())),
      generateRamseteCommandFromTrajectory(manualTrajectory1),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
      new InstantCommand(() -> launcher.setLauncherForPosition(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> collector.intake(), collector),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorStop(), collector)
    );

    fourBallAutoCommandGroup = new SequentialCommandGroup(
      new InstantCommand(() -> System.out.println("Running Partial Auto")),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
      new InstantCommand(() -> launcher.setLauncherForPosition(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> collector.feederOff(), collector),
      new TurnForDegrees(155, drive),
      new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
      new InstantCommand(() -> collector.intake(), collector),
      new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory1.getInitialPose())),
      generateRamseteCommandFromTrajectory(fourBallAutoTrajectory1),
      new WaitCommand(0.25),
      new InstantCommand(() -> collector.moverOff(), collector),
      new TurnForDegrees(105, drive),
      new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory2.getInitialPose())),
      generateRamseteCommandFromTrajectory(fourBallAutoTrajectory2),
      new TurnForDegrees(115, drive),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() ->  collector.feederOn(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(() -> collector.moverOff(), collector),
      new TurnForDegrees(180, drive),
       new WaitCommand(0.25),
       new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory3.getInitialPose())),
       generateRamseteCommandFromTrajectory(fourBallAutoTrajectory3),
       new TurnForDegrees(160, drive),
       new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory4.getInitialPose())),
       generateRamseteCommandFromTrajectory(fourBallAutoTrajectory4),
      // new InstantCommand(() -> launcher.feederOn(), launcher),
      // new WaitCommand(0.5),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorStop(), collector),
      new InstantCommand(()-> collector.stop())
    );

    autoCommandChooser.setDefaultOption("Two Ball Auto Right", twoBallAutoCommandGroupRight);
    autoCommandChooser.addOption("Two Ball Auto Left", twoBallAutoCommandGroupLeft);
    autoCommandChooser.addOption("Four Ball", fourBallAutoCommandGroup);

    allianceColor.setDefaultOption("Blue", BallColor.BLUE);
    allianceColor.addOption("Red", BallColor.RED);

    SmartDashboard.putData("Auto Command Chooser", autoCommandChooser);
    SmartDashboard.putData("Alliance Color", allianceColor);

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

    driveLeftBumper.whenPressed(new InstantCommand(() -> climber.winchDown(), climber));
    driveLeftBumper.whenReleased(new InstantCommand(() -> climber.stop(), climber));

    driveRightBumper.whenPressed(new InstantCommand(() -> climber.winchUp(), climber));
    driveRightBumper.whenReleased(new InstantCommand(() -> climber.stop(), climber));

    driveA.whenPressed(new InstantCommand(() -> climber.toggleSolenoid(), climber));
    
    driveB.whenPressed(new InstantCommand(() -> climber.setWinch(0.1), climber));
    driveB.whenReleased(new InstantCommand(() -> climber.setWinch(0), climber));

    //driveA.whenPressed(climbForDistance);
    //driveA.whenReleased(() -> climbForDistance.cancel());

    driveBack.whenPressed(new InstantCommand(() -> climber.setLeft(-0.4), climber));
    driveBack.whenReleased(new InstantCommand(() -> climber.setLeft(0), climber));

    driveStart.whenPressed(new InstantCommand(() -> climber.setRight(0.4), climber));
    driveStart.whenReleased(new InstantCommand(() -> climber.setRight(0), climber));

    driveX.whenPressed(new InstantCommand(() -> climber.setLeft(0.4), climber));
    driveX.whenReleased(new InstantCommand(() -> climber.setLeft(0), climber));

    driveY.whenPressed(new InstantCommand(() -> climber.setRight(-0.4), climber));
    driveY.whenReleased(new InstantCommand(() -> climber.setRight(0), climber));

    driveLeftJoystickButton.whileHeld(collectForDistance);
    //driveLeftJoystickButton.whenReleased(() -> collector.cancel());

    // opY.whenPressed(new InstantCommand(() -> launcher.setLauncherForPosition()));
    // opY.whenReleased(new InstantCommand(() -> launcher.stop()));

    opY.whenPressed(new InstantCommand(() -> launcher.pidOn(), launcher));
    opY.whenReleased(new InstantCommand(() -> launcher.pidOff(), launcher));

    // opY.whenPressed(new InstantCommand(() -> launcher.setLauncher(0.23, 0.30), launcher));
    // opY.whenReleased(new InstantCommand(() -> launcher.setLauncher(0.00, 0.00), launcher));

    // opY.whenPressed(new InstantCommand(() -> launcher.setLauncher(1, 1), launcher));
    // opY.whenReleased(new InstantCommand(() -> launcher.setLauncher(0, 0), launcher));

    //opB.whenPressed(new InstantCommand(() -> collector.feederOn()));
    opB.whileHeld(new InstantCommand(() -> collector.smartBallShoot(), collector));
    opB.whenReleased(new InstantCommand(() -> {
      collector.feederOff();
      collector.moverOff();
    }, collector));
    

    opA.whenPressed(new InstantCommand(() -> collector.toggleSolenoid(), collector));

    opLeftBumper.whenPressed(new InstantCommand(() -> collector.intake(), collector));
    opLeftBumper.whenReleased(new InstantCommand(() -> collector.stop(), collector));

    opRightBumper.whenPressed(new InstantCommand(() -> collector.outake(), collector));
    opRightBumper.whenReleased(new InstantCommand(() -> collector.stop(), collector));
  }

  /**
   * Master method for updating the updateShuffleboard() method in each subsystem
   */
  public void updateShuffleboard() {
    //drive.updateShuffleboard();
    launcher.updateShuffleboard();
    colorSensor.updateShuffleboard();
    //climber.updateShuffleboard();
    collector.updateShuffleboard();
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

    try {
      manualTrajectory1 = generateTrajectoryFromJSON(manualPath1);
      fourBallAutoTrajectory1 = generateTrajectoryFromJSON(fourBallAutoPath1);
      fourBallAutoTrajectory2 = generateTrajectoryFromJSON(fourBallAutoPath2);
      fourBallAutoTrajectory3 = generateTrajectoryFromJSON(fourBallAutoPath3);
      fourBallAutoTrajectory4 = generateTrajectoryFromJSON(fourBallAutoPath4);


    } catch (IOException e) {
      System.out.println("Could not read trajectory file.");
    }
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
            drive);
  }

  public BallColor getSelectedAllianceColor() {
    return allianceColor.getSelected();
  }

  public ColorSensor getColorSensor() {
    return colorSensor;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(){
    // Run path following command, then stop at the end.
    return autoCommandChooser.getSelected();
  }
}
