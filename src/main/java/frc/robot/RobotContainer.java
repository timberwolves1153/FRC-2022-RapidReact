// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ClimbForDistance;
import frc.robot.commands.CollectForDistance;
import frc.robot.commands.DefaultCollect;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DefaultLauncher;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnForDegrees;
import frc.robot.commands.TurnWithLimeLight;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.WinchDown;
import frc.robot.commands.commandGroups.TwoBallAutoLeftGroup;
import frc.robot.commands.commandGroups.TwoBallAutoRightGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Rumble;
import frc.robot.subsystems.ColorSensor.BallColor;
import frc.robot.subsystems.Launcher.ShooterPosition;
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

  private POVButton opPovUp;
  private POVButton opPovDown;
  private POVButton opPovRight;
  private POVButton opPovLeft;

  private JoystickButton opY;
  private JoystickButton opB;
  private JoystickButton opA;
  private JoystickButton opLeftBumper;
  private JoystickButton opRightBumper;
  private JoystickButton opStart;

  private ClimbForDistance climbForDistance;
  private CollectForDistance collectForDistance;
  //private SmartShoot smartShoot;
  private WinchDown winchDownCommand;
  private TurnWithLimeLight turnWithLimeLight;
  private Shoot shoot;
  
  //Instantiates all subsystems
  private Drive drive;
  private Collector collector;
  private Climber climber;
  private Launcher launcher;
  private ColorSensor colorSensor;
  private LEDLights ledLights;
  private Limelight limelight;
  private Rumble rumble;

  private Trajectory fourBallAutoTrajectory1;
  private Trajectory fourBallAutoTrajectory2;
  private Trajectory fourBallAutoTrajectory3;
  private Trajectory fourBallAutoTrajectory4;
  private Trajectory fourBallAutoTrajectory5;

  private Trajectory manualTrajectory1;

  private Trajectory gatewayPathTrajectory1;
  private Trajectory gatewayPathTrajectory2;
  private Trajectory gatewayPathTrajectory3;

  private Trajectory threeBallAutoTrajectory3;
 
  private SequentialCommandGroup twoBallAutoCommandGroupRight;
  private SequentialCommandGroup twoBallAutoCommandGroupLeft;
  private SequentialCommandGroup fourBallAutoCommandGroup;
  private SequentialCommandGroup gateKeepAutoCommandGroup;
  private SequentialCommandGroup threeBallAutoCommandGroup;

  private String manualPath1 = "pathplanner/generatedJSON/ManualPath1.wpilib.json";
  private String fourBallAutoPath1 = "pathplanner/generatedJSON/FourBallAutoPath1.wpilib.json";
  private String fourBallAutoPath2 = "pathplanner/generatedJSON/FourBallAutoPath2.wpilib.json";
  private String fourBallAutoPath3 = "pathplanner/generatedJSON/FourBallAutoPath3.wpilib.json";
  private String fourBallAutoPath4 = "pathplanner/generatedJSON/FourBallAutoPath4.wpilib.json";
  private String fourBallAutoPath5 = "pathplanner/generatedJSON/FourBallAutoPath5.wpilib.json";
  private String gatewayPath1 = "pathplanner/generatedJSON/GatewayPath1.wpilib.json";
  private String gatewayPath2 = "pathplanner/generatedJSON/GatewayPath2.wpilib.json";
  private String gatewayPath3 = "pathplanner/generatedJSON/GatewayPath3.wpilib.json";
  private String threeBallAutoPath3 = "pathplanner/generatedJSON/ThreeBallAutoPath3.wpilib.json";

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
    limelight = new Limelight();
    rumble = new Rumble(driveStick, opStick);

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
    opStart = new JoystickButton(opStick, XboxController.Button.kStart.value);
    opLeftBumper = new JoystickButton(opStick, XboxController.Button.kLeftBumper.value);
    opRightBumper = new JoystickButton(opStick, XboxController.Button.kRightBumper.value);
    opPovUp = new POVButton(opStick, 0);
    opPovDown = new POVButton(opStick, 180);
    opPovLeft = new POVButton(opStick, 270);
    opPovRight = new POVButton(opStick, 90);

    turnWithLimeLight = new TurnWithLimeLight(drive, limelight);
    climbForDistance = new ClimbForDistance(5, climber);
    collectForDistance = new CollectForDistance(5, collector);
    //smartShoot = new SmartShoot(collector, colorSensor, launcher);
    winchDownCommand = new WinchDown(climber);
    shoot = new Shoot(launcher, limelight);

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
    
    /*twoBallAutoCommandGroupRight = new SequentialCommandGroup(
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
      new InstantCommand(() -> collector.collectIntake(), collector),
      new InstantCommand(()-> drive.resetOdometry(manualTrajectory1.getInitialPose())),
      generateRamseteCommandFromTrajectory(manualTrajectory1),
      new TurnForDegrees(185, drive),
      new InstantCommand(()-> drive.resetOdometry(manualTrajectory1.getInitialPose())),
      generateRamseteCommandFromTrajectory(manualTrajectory1),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
      new InstantCommand(() -> launcher.setLauncherForPosition(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> collector.collectorStop(), collector),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorStop(), collector)
    );*/

    twoBallAutoCommandGroupRight = new TwoBallAutoRightGroup(
      manualTrajectory1, 
      () -> generateRamseteCommandFromTrajectory(manualTrajectory1), 
      launcher, collector, drive);

    /*twoBallAutoCommandGroupLeft = new SequentialCommandGroup(
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
      new InstantCommand(() -> collector.collectorStop(), collector),
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
      new InstantCommand(() -> collector.collectorStop(), collector),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorStop(), collector)
    );*/

    twoBallAutoCommandGroupLeft = new TwoBallAutoLeftGroup(
      manualTrajectory1, 
      () -> generateRamseteCommandFromTrajectory(manualTrajectory1), 
      launcher, collector, drive);

    fourBallAutoCommandGroup = new SequentialCommandGroup(
      new InstantCommand(() -> System.out.println("Running Partial Auto")),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
      new InstantCommand(() -> launcher.pidOn(), launcher),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.collectorStop(), collector),
      new WaitCommand(1),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.HALF_COURT), launcher),
      new TurnForDegrees(180, drive),
      new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
      new InstantCommand(() -> collector.collectIntake(), collector),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory1.getInitialPose())),
      generateRamseteCommandFromTrajectory(fourBallAutoTrajectory1),
      //new WaitCommand(0.25),
      new TurnForDegrees(95, drive),
     // new InstantCommand(() -> collector.moverOff(), collector),
     // new TurnForDegrees(185, drive),
      // new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.TARMAC_HIGH), launcher),
      // new InstantCommand(() -> launcher.setLauncherForPosition(), launcher),
      //new InstantCommand(() -> collector.feederOn(), collector),
      //new WaitCommand(2),
      //new TurnForDegrees(-85, drive),
      //new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory2.getInitialPose())),
      generateRamseteCommandFromTrajectory(fourBallAutoTrajectory2),
      new TurnForDegrees(100, drive),
      new InstantCommand(() -> collector.collectorStop(), collector),
      new InstantCommand(() -> launcher.pidOn(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() ->  collector.feederOn(), collector),
      new WaitCommand(1),
      new InstantCommand(() -> launcher.pidOff(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorStop(), collector),
      new InstantCommand(()-> collector.collectorStop(), collector)
    
    //   new InstantCommand(() -> collector.feederOff(), collector),
    //   new InstantCommand(() -> collector.moverOff(), collector),
    //   new TurnForDegrees(180, drive),
    //    new WaitCommand(0.25),
    //    new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory3.getInitialPose())),
    //    generateRamseteCommandFromTrajectory(fourBallAutoTrajectory3),
    //    new TurnForDegrees(160, drive),
    //   new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory4.getInitialPose())),
    //    generateRamseteCommandFromTrajectory(fourBallAutoTrajectory4),
    //   new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.TARMAC_HIGH), launcher),
    //   new InstantCommand(() -> launcher.pidOn(), launcher),
    //   new InstantCommand(() -> collector.moverForward(), collector),
    //   new InstantCommand(() -> collector.feederOn(), collector),
    //   new WaitCommand(1),
    //   new InstantCommand(() -> launcher.pidOff(), launcher),
    //   new InstantCommand(() -> collector.feederOff(), collector),
    //   new InstantCommand(()-> collector.moverOff(), collector),
    //   new InstantCommand(()-> collector.singulatorStop(), collector),
    //   new InstantCommand(()-> collector.collectorStop(), collector)
     );

    threeBallAutoCommandGroup = new SequentialCommandGroup(
      new InstantCommand(()-> System.out.println("Running Three Ball Auto")),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.DEAD_ZONE), launcher),
      new InstantCommand(() -> launcher.pidOn(), launcher),
      new InstantCommand(()-> collector.feederOn(), collector),
      new WaitCommand(1),
      new InstantCommand(()-> collector.feederOff(), collector),
      new TurnForDegrees(170, drive),
      new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new InstantCommand(() -> collector.collectIntake(), collector),
      new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory1.getInitialPose())),
      generateRamseteCommandFromTrajectory(fourBallAutoTrajectory1),
      new TurnForDegrees(105, drive),
      new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory2.getInitialPose())),
      generateRamseteCommandFromTrajectory(fourBallAutoTrajectory2),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.collectorStop(), collector),
      new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kForward)),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.TARMAC_HIGH), launcher),
      new TurnForDegrees(95, drive), 
     // new TurnWithLimeLight(drive, limelight),
      //generateRamseteCommandFromTrajectory(threeBallAutoTrajectory3),
      new InstantCommand(()-> collector.moverForward(), collector),
      new InstantCommand(()-> collector.feederOn(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> launcher.pidOff(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(()-> collector.moverOff(), collector),
      new InstantCommand(()-> collector.singulatorStop(), collector),
      new InstantCommand(()-> collector.collectorStop(), collector)
    );

    gateKeepAutoCommandGroup = new SequentialCommandGroup(
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
      new InstantCommand(() -> launcher.pidOn(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(2),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new TurnForDegrees(180, drive),
      new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
      new InstantCommand(() -> collector.collectIntake(), collector),
      new InstantCommand(()-> drive.resetOdometry(gatewayPathTrajectory1.getInitialPose())),
      generateRamseteCommandFromTrajectory(gatewayPathTrajectory1),
      new TurnForDegrees(170, drive),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.TARMAC_HIGH), launcher),
      new InstantCommand(() -> launcher.pidOn(), launcher),
      new InstantCommand(() -> collector.moverForward(), collector),
      new InstantCommand(() -> collector.feederOn(), collector),
      new InstantCommand(() -> collector.singulatorIntake(), collector),
      new WaitCommand(0.5),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(() -> collector.moverOff(), collector),
      new TurnForDegrees(-60, drive),
      new InstantCommand(()-> drive.resetOdometry(gatewayPathTrajectory2.getInitialPose())),
      generateRamseteCommandFromTrajectory(gatewayPathTrajectory2),
      new TurnForDegrees(-120, drive),
      new InstantCommand(() -> collector.collectOutake(), collector),
      new InstantCommand(() -> collector.singulatorOutake(), collector),
      new InstantCommand(() -> collector.moverReverse(), collector),
      new WaitCommand(0.25),
      new TurnForDegrees(-50, drive),
      new InstantCommand(() -> drive.resetOdometry(gatewayPathTrajectory3.getInitialPose())),
      generateRamseteCommandFromTrajectory(gatewayPathTrajectory3),
      new WaitCommand(0.5),
      new InstantCommand(() -> collector.collectOutake(), collector),
      new InstantCommand(() -> collector.singulatorOutake(), collector),
      new InstantCommand(() -> collector.moverReverse(), collector),
      new WaitCommand(1),
      new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
      new InstantCommand(() -> collector.collectIntake(), collector),
      new InstantCommand(() -> launcher.stop(), launcher),
      new InstantCommand(() -> collector.feederOff(), collector),
      new InstantCommand(() -> collector.moverOff(), collector),
      new InstantCommand(() -> collector.singulatorStop(), collector)
    );
      
    autoCommandChooser.setDefaultOption("Two Ball Auto Right", twoBallAutoCommandGroupRight);
    autoCommandChooser.addOption("Two Ball Auto Left", twoBallAutoCommandGroupLeft);
    autoCommandChooser.addOption("Four Ball", fourBallAutoCommandGroup);
    autoCommandChooser.addOption("Three Ball", threeBallAutoCommandGroup);
    autoCommandChooser.addOption("Gatekeep", gateKeepAutoCommandGroup);

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

    driveLeftBumper.whenPressed(winchDownCommand);
    driveLeftBumper.whenReleased(() -> winchDownCommand.cancel());

    driveRightBumper.whenPressed(new InstantCommand(() -> climber.winchUp(), climber));
    driveRightBumper.whenReleased(new InstantCommand(() -> climber.stop(), climber));

    driveA.whenPressed(new InstantCommand(() -> climber.toggleSolenoid(), climber));
    
    driveB.whenPressed(new InstantCommand(() -> climber.setWinchPosition(climber.getEncoderTicksFromPosition(12)), climber));
    driveB.whenReleased(new InstantCommand(() -> climber.setWinch(0), climber));

    //driveA.whenPressed(climbForDistance);
    //driveA.whenReleased(() -> climbForDistance.cancel());

    driveBack.whenPressed(new InstantCommand(() -> climber.setLeft(0.4), climber));
    driveBack.whenReleased(new InstantCommand(() -> climber.setLeft(0), climber));

    driveStart.whenPressed(new InstantCommand(() -> climber.setRight(0.4), climber));
    driveStart.whenReleased(new InstantCommand(() -> climber.setRight(0), climber));

    driveX.whenPressed(new InstantCommand(() -> climber.setLeft(-0.4), climber));
    driveX.whenReleased(new InstantCommand(() -> climber.setLeft(0), climber));

    driveY.whenPressed(new InstantCommand(() -> climber.setRight(-0.4), climber));
    driveY.whenReleased(new InstantCommand(() -> climber.setRight(0), climber));

    driveRightJoystickButton.whenPressed(turnWithLimeLight);
    driveRightJoystickButton.whenReleased(() -> turnWithLimeLight.cancel());

    driveLeftJoystickButton.whileHeld(collectForDistance);
    //driveLeftJoystickButton.whenReleased(() -> collector.cancel());

    // opY.whenPressed(new InstantCommand(() -> launcher.setLauncherForPosition()));
    // opY.whenReleased(new InstantCommand(() -> launcher.stop()));

    //  opY.whenPressed(new InstantCommand(() -> launcher.pidOn(), launcher));
    //  opY.whenReleased(new InstantCommand(() -> launcher.pidOff(), launcher));

    opY.whenPressed(shoot);
    opY.whenReleased(() -> shoot.cancel());

    opStart.whenPressed(() -> launcher.toggleLimelightOverride());

    opPovUp.whenPressed(new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.UPPER_HUB), launcher));
    opPovDown.whenPressed(new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.LOWER_HUB), launcher));
    opPovLeft.whenPressed(new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.DEAD_ZONE), launcher));
    opPovRight.whenPressed(new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.TARMAC_HIGH), launcher));

    opLeftBumper.whenPressed(new InstantCommand(() -> {
      collector.collectIntake();
      collector.setSolenoid(DoubleSolenoid.Value.kReverse);
    }, collector));

    opLeftBumper.whenReleased(new InstantCommand(() -> {
      collector.collectorStop();
      collector.setSolenoid(DoubleSolenoid.Value.kForward);
    }, collector));


    opRightBumper.whenPressed(new InstantCommand(() -> {
      collector.collectOutake();
      collector.setSolenoid(DoubleSolenoid.Value.kReverse);
    }, collector));

    opRightBumper.whenReleased(new InstantCommand(() -> {
      collector.collectorStop();
      collector.setSolenoid(DoubleSolenoid.Value.kForward);
    }, collector));

    // opY.whenPressed(new InstantCommand(() -> launcher.setLauncher(0.23, 0.30), launcher));
    // opY.whenReleased(new InstantCommand(() -> launcher.setLauncher(0.00, 0.00), launcher));

    // opY.whenPressed(new InstantCommand(() -> launcher.setLauncher(1, 1), launcher));
    // opY.whenReleased(new InstantCommand(() -> launcher.setLauncher(0, 0), launcher));

    // opB.whenPressed(new InstantCommand(() -> collector.feederOn()));
    // opB.whileHeld(new InstantCommand(() -> {
    //   collector.smartBallShoot();
    //   launcher.smart();
    // }, collector));
    // opB.whenReleased(new InstantCommand(() -> {
    //   collector.feederOff();
    //   collector.moverOff();
    // }, collector, launcher));

    //  opY.whenPressed(smartShoot);
    //  opY.whenReleased(() -> smartShoot.cancel());
    
    opA.whenPressed(new InstantCommand(() -> collector.toggleSolenoid(), collector));

    opLeftBumper.whenPressed(new InstantCommand(() -> collector.collectIntake(), collector));
    opLeftBumper.whenReleased(new InstantCommand(() -> collector.collectorStop(), collector));

    opRightBumper.whenPressed(new InstantCommand(() -> collector.collectOutake(), collector));
    opRightBumper.whenReleased(new InstantCommand(() -> collector.collectorStop(), collector));
    //change back to while held when smart shooting is being used.
    opB.whenPressed(new InstantCommand(()-> collector.feederOn(), collector));
    opB.whenReleased(new InstantCommand(()-> collector.feederOff(), collector));

  }

  /**
   * Master method for updating the updateShuffleboard() method in each subsystem
   */
  public void updateShuffleboard() {
    //drive.updateShuffleboard();
    launcher.updateShuffleboard();
    colorSensor.updateShuffleboard();
    climber.updateShuffleboard();
    //collector.updateShuffleboard();
    limelight.updateShuffleBoard();
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
      threeBallAutoTrajectory3 = generateTrajectoryFromJSON(threeBallAutoPath3);
      gatewayPathTrajectory1 = generateTrajectoryFromJSON(gatewayPath1);
      gatewayPathTrajectory2 = generateTrajectoryFromJSON(gatewayPath2);
      gatewayPathTrajectory3 = generateTrajectoryFromJSON(gatewayPath3);
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
    return new RamseteCommand(
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
        drive
      );
  }

  public BallColor getSelectedAllianceColor() {
    return allianceColor.getSelected();
  }

  public ColorSensor getColorSensor() {
    return colorSensor;
  }

  public Launcher getLauncher() {
    return launcher;
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
