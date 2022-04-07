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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ClimbForDistance;
import frc.robot.commands.DefaultCollect;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DefaultLauncher;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnWithLimeLight;
import frc.robot.commands.TurnWithLimelightV2;
import frc.robot.commands.WinchDown;
import frc.robot.commands.commandGroups.FiveBallAutoGroup;
import frc.robot.commands.commandGroups.FourBallAutoGroup;
import frc.robot.commands.commandGroups.GatekeepAutoGroup;
import frc.robot.commands.commandGroups.ThreeBallAutoGroup;
import frc.robot.commands.commandGroups.TwoBallAutoLeftGroup;
import frc.robot.commands.commandGroups.TwoBallAutoRightGroup;
import frc.robot.lib.ShooterPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Rumble;
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
  //private SmartShoot smartShoot;
  private WinchDown winchDownCommand;
  private TurnWithLimelightV2 turnWithLimeLight;
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
  private Trajectory fiveBallAutoTrajectory1;
  private Trajectory fiveBallAutoTrajectory2;
  private Trajectory fiveBallAutoTrajectory3;

  private Trajectory manualTrajectory1;

  private Trajectory gatekeepPathTrajectory1;
  private Trajectory gatekeepPathTrajectory2;
  private Trajectory gatekeepPathTrajectory3;

  private Trajectory threeBallAutoTrajectory3;
  private Trajectory tuningTrajectory;

  private Trajectory tuningTrajectory2;
 
  private SequentialCommandGroup twoBallAutoCommandGroupRight;
  private SequentialCommandGroup twoBallAutoCommandGroupLeft;
  private SequentialCommandGroup fourBallAutoCommandGroup;
  private SequentialCommandGroup gatekeepAutoCommandGroup;
  private SequentialCommandGroup threeBallAutoCommandGroup;
  private SequentialCommandGroup fiveBallAutoCommandGroup;

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
  private String fiveBallAutoPath1 = "pathplanner/generatedJSON/FiveBallAutoPath1.wpilib.json";
  private String fiveBallAutoPath2 = "pathplanner/generatedJSON/FiveBallAutoPath2.wpilib.json";
  private String fiveBallAutoPath3 = "pathplanner/generatedJSON/FiveBallAutoPath3.wpilib.json";
  private String tuningPath = "pathplanner/generatedJSON/TuningPath.wpilib.json";

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

    turnWithLimeLight = new TurnWithLimelightV2(drive, limelight);
    climbForDistance = new ClimbForDistance(5, climber);
    //smartShoot = new SmartShoot(collector, colorSensor, launcher);
    winchDownCommand = new WinchDown(climber);
    shoot = new Shoot(false, launcher, limelight);

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

    generateCommandGroups();
      
    autoCommandChooser.setDefaultOption("Two Ball Auto Right", twoBallAutoCommandGroupRight);
    autoCommandChooser.addOption("Two Ball Auto Left", twoBallAutoCommandGroupLeft);
    autoCommandChooser.addOption("Four Ball", fourBallAutoCommandGroup);
    autoCommandChooser.addOption("Three Ball", threeBallAutoCommandGroup);
    autoCommandChooser.addOption("Gatekeep", gatekeepAutoCommandGroup);
    autoCommandChooser.addOption("Five Ball", fiveBallAutoCommandGroup);
    autoCommandChooser.addOption("Tuning Path", new SequentialCommandGroup(
      new InstantCommand(() -> drive.resetOdometry(tuningTrajectory2.getInitialPose()), drive),
      generateRamseteCommandFromTrajectory(tuningTrajectory2)
    ));

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
    driveLeftBumper.whenPressed(winchDownCommand);
    driveLeftBumper.whenReleased(() -> winchDownCommand.cancel());

    driveRightBumper.whenPressed(new InstantCommand(() -> climber.winchUp(), climber));
    driveRightBumper.whenReleased(new InstantCommand(() -> climber.stop(), climber));

    driveA.whenPressed(new InstantCommand(() -> climber.toggleSolenoid(), climber));
    
    driveB.whenPressed(new InstantCommand(() -> climber.setWinchPosition(climber.getEncoderTicksFromPosition(12)), climber));
    driveB.whenReleased(new InstantCommand(() -> climber.setWinch(0), climber));

    driveBack.whenPressed(new InstantCommand(() -> climber.setLeft(0.4), climber));
    driveBack.whenReleased(new InstantCommand(() -> climber.setLeft(0), climber));

    driveStart.whenPressed(new InstantCommand(() -> climber.setRight(0.4), climber));
    driveStart.whenReleased(new InstantCommand(() -> climber.setRight(0), climber));

    driveX.whenPressed(new InstantCommand(() -> climber.setLeft(-0.4), climber));
    driveX.whenReleased(new InstantCommand(() -> climber.setLeft(0), climber));

    driveY.whenPressed(new InstantCommand(() -> climber.setRight(-0.4), climber));
    driveY.whenReleased(new InstantCommand(() -> climber.setRight(0), climber));

    // driveRightJoystickButton.whileHeld(new InstantCommand(() -> drive.turnWithLimelight(limelight), drive));
    // driveRightJoystickButton.whenReleased(new InstantCommand(() -> drive.arcadeDrive(0, 0), drive));

    driveRightJoystickButton.whenPressed(turnWithLimeLight);
    driveRightJoystickButton.whenReleased(() -> turnWithLimeLight.cancel());

    // driveLeftJoystickButton.whenPressed(new InstantCommand(() -> drive.resetOdometry(new Pose2d(8.74, 5.54, new Rotation2d(69.34 * (Math.PI / 180))))));
    driveLeftJoystickButton.whenPressed(new InstantCommand(() -> drive.resetOdometry(new Pose2d(0, 0, new Rotation2d()))));

    opY.whenPressed(shoot);
    opY.whenReleased(() -> shoot.cancel());

    opStart.whenPressed(() -> launcher.toggleLimelightOverride());

    opPovUp.whenPressed(new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.FENDER_HIGH)));
    opPovDown.whenPressed(new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.FENDER_LOW)));
    opPovLeft.whenPressed(new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.TARMAC_ZONE)));
    opPovRight.whenPressed(new InstantCommand(() -> launcher.setGainPreset(ShooterPosition.TARMAC_LINE_HIGH)));

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

    // opB.whenPressed(new InstantCommand(() -> collector.feederOn()));
    // opB.whileHeld(new InstantCommand(() -> {
    //   collector.smartBallShoot();
    //   launcher.smart();
    // }, collector));
    // opB.whenReleased(new InstantCommand(() -> {
    //   collector.feederOff();
    //   collector.moverOff();
    // }, collector, launcher));
    
    opA.whenPressed(new InstantCommand(() -> collector.toggleSolenoid(), collector));

    opLeftBumper.whenPressed(new InstantCommand(() -> collector.collectIntake(), collector));
    opLeftBumper.whenReleased(new InstantCommand(() -> collector.collectorStop(), collector));

    opRightBumper.whenPressed(new InstantCommand(() -> collector.collectOutake(), collector));
    opRightBumper.whenReleased(new InstantCommand(() -> collector.collectorStop(), collector));

    //change back to while held when smart shooting is being used.
    opB.whenPressed(new InstantCommand(()-> collector.feederOn(), collector));
    opB.whenReleased(new InstantCommand(()-> collector.feederOff(), collector));
  }

  private void generateCommandGroups() {
    twoBallAutoCommandGroupRight = new TwoBallAutoRightGroup(
      manualTrajectory1, 
      () -> generateRamseteCommandFromTrajectory(manualTrajectory1), 
      launcher, collector, drive);

    twoBallAutoCommandGroupLeft = new TwoBallAutoLeftGroup(
      fourBallAutoTrajectory1, 
      () -> generateRamseteCommandFromTrajectory(fourBallAutoTrajectory1), 
      launcher, collector, drive);

    fourBallAutoCommandGroup = new FourBallAutoGroup(
      fourBallAutoTrajectory1, 
      fourBallAutoTrajectory2, 
      fourBallAutoTrajectory4,
      () -> generateRamseteCommandFromTrajectory(fourBallAutoTrajectory1), 
      () -> generateRamseteCommandFromTrajectory(fourBallAutoTrajectory2), 
      () -> generateRamseteCommandFromTrajectory(fourBallAutoTrajectory4), 
      collector, 
      launcher, 
      drive
    );

    threeBallAutoCommandGroup = new ThreeBallAutoGroup(
      fourBallAutoTrajectory1, 
      fourBallAutoTrajectory2, 
      fourBallAutoTrajectory3,
      () -> generateRamseteCommandFromTrajectory(fourBallAutoTrajectory1), 
      () -> generateRamseteCommandFromTrajectory(fourBallAutoTrajectory2), 
      () -> generateRamseteCommandFromTrajectory(fourBallAutoTrajectory3),
      collector, 
      drive, 
      launcher
    );

    gatekeepAutoCommandGroup = new GatekeepAutoGroup(
      gatekeepPathTrajectory1, 
      gatekeepPathTrajectory2, 
      gatekeepPathTrajectory3,
      () -> generateRamseteCommandFromTrajectory(gatekeepPathTrajectory1), 
      () -> generateRamseteCommandFromTrajectory(gatekeepPathTrajectory2),
      () -> generateRamseteCommandFromTrajectory(gatekeepPathTrajectory3),
      collector, 
      drive, 
      launcher
    );

    fiveBallAutoCommandGroup = new FiveBallAutoGroup(
      fiveBallAutoTrajectory1, 
      fiveBallAutoTrajectory2, 
      fiveBallAutoTrajectory3,
      () -> generateRamseteCommandFromTrajectory(fiveBallAutoTrajectory1), 
      () -> generateRamseteCommandFromTrajectory(fiveBallAutoTrajectory2),
      () -> generateRamseteCommandFromTrajectory(fiveBallAutoTrajectory3),
      collector, 
      launcher,
      drive
    );

    // threeBallAutoCommandGroup = new SequentialCommandGroup(
    //   new InstantCommand(()-> System.out.println("Running Three Ball Auto")),
    //   new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
    //   new InstantCommand(() -> launcher.pidOn(), launcher),
    //   new InstantCommand(() -> collector.moverForward(), collector),
    //   new InstantCommand(()-> collector.feederOn(), collector),
    //   new WaitCommand(1),
    //   new InstantCommand(()-> collector.feederOff(), collector),
    //   new InstantCommand(()-> collector.moverOff(), collector),
    //   new TurnForDegrees(170, drive),
    //   new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
    //   new InstantCommand(() -> collector.moverForward(), collector),
    //   new InstantCommand(() -> collector.singulatorIntake(), collector),
    //   new InstantCommand(() -> collector.collectIntake(), collector),
    //   new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory1.getInitialPose())),
    //   generateRamseteCommandFromTrajectory(fourBallAutoTrajectory1),
    //   new TurnForDegrees(100, drive),
    //   new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory2.getInitialPose())),
    //   generateRamseteCommandFromTrajectory(fourBallAutoTrajectory2),
    //   new InstantCommand(()-> collector.moverOff(), collector),
    // //  new InstantCommand(()-> collector.collectorStop(), collector),
    //  // new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kForward)),
    //   new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.TARMAC_HIGH), launcher),
    //   new TurnForDegrees(105, drive), 
    //   new InstantCommand(()-> drive.resetOdometry(fourBallAutoTrajectory3.getInitialPose())),
    //   generateRamseteCommandFromTrajectory(fourBallAutoTrajectory3),
    //  // new TurnWithLimeLight(drive, limelight),
    //   //generateRamseteCommandFromTrajectory(threeBallAutoTrajectory3),
    //   new InstantCommand(()-> collector.moverForward(), collector),
    //   new InstantCommand(()-> collector.feederOn(), collector),
    //   new WaitCommand(2),
    //   new InstantCommand(() -> launcher.pidOff(), launcher),
    //   new InstantCommand(() -> collector.feederOff(), collector),
    //   new InstantCommand(()-> collector.moverOff(), collector),
    //   new InstantCommand(()-> collector.singulatorStop(), collector),
    //   new InstantCommand(()-> collector.collectorStop(), collector)
    // );

    // gateKeepAutoCommandGroup = new SequentialCommandGroup(
    //   new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
    //   new InstantCommand(() -> launcher.pidOn(), launcher),
    //   new InstantCommand(() -> collector.moverForward(), collector),
    //   new InstantCommand(() -> collector.feederOn(), collector),
    //   new InstantCommand(() -> collector.singulatorIntake(), collector),
    //   new WaitCommand(2),
    //   new InstantCommand(() -> launcher.stop(), launcher),
    //   new InstantCommand(() -> collector.feederOff(), collector),
    //   new TurnForDegrees(185, drive),
    //   new InstantCommand(() -> collector.setSolenoid(DoubleSolenoid.Value.kReverse)),
    //   new InstantCommand(() -> collector.collectIntake(), collector),
    //   new InstantCommand(()-> drive.resetOdometry(gatewayPathTrajectory1.getInitialPose())),
    //   generateRamseteCommandFromTrajectory(gatewayPathTrajectory1),
    //   new TurnForDegrees(160, drive),
    //   new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.TARMAC_HIGH), launcher),
    //   new InstantCommand(() -> launcher.pidOn(), launcher),
    //   new InstantCommand(() -> collector.moverForward(), collector),
    //   new InstantCommand(() -> collector.feederOn(), collector),
    //   new InstantCommand(() -> collector.singulatorIntake(), collector),
    //   new WaitCommand(1.5),
    //   new InstantCommand(() -> launcher.stop(), launcher),
    //   new InstantCommand(() -> collector.feederOff(), collector),
    //   new InstantCommand(() -> collector.moverOff(), collector),
    //   new TurnForDegrees(-85, drive),
    //   new InstantCommand(()-> drive.resetOdometry(gatewayPathTrajectory2.getInitialPose())),
    //   generateRamseteCommandFromTrajectory(gatewayPathTrajectory2),
    //   new TurnForDegrees(-120, drive),
    //   new InstantCommand(() -> collector.collectOutake(), collector),
    //   new InstantCommand(() -> collector.singulatorOutake(), collector),
    //   new InstantCommand(() -> collector.moverReverse(), collector),
    //   new WaitCommand(0.25),
    //   new TurnForDegrees(-50, drive),
    //   new InstantCommand(() -> drive.resetOdometry(gatewayPathTrajectory3.getInitialPose())),
    //   generateRamseteCommandFromTrajectory(gatewayPathTrajectory3),
    //   new WaitCommand(0.5),
    //   new InstantCommand(() -> collector.collectOutake(), collector),
    //   new InstantCommand(() -> collector.singulatorOutake(), collector),
    //   new InstantCommand(() -> collector.moverReverse(), collector),
    //   new WaitCommand(1),
    //   new InstantCommand(() -> launcher.setGainPreset(Launcher.ShooterPosition.UPPER_HUB), launcher),
    //   new InstantCommand(() -> collector.collectIntake(), collector),
    //   new InstantCommand(() -> launcher.stop(), launcher),
    //   new InstantCommand(() -> collector.feederOff(), collector),
    //   new InstantCommand(() -> collector.moverOff(), collector),
    //   new InstantCommand(() -> collector.singulatorStop(), collector)
    // );
  }

  /**
   * Master method for updating the updateShuffleboard() method in each subsystem
   */
  public void updateShuffleboard() {
    drive.updateShuffleboard();
    //launcher.updateShuffleboard();
    //colorSensor.updateShuffleboard();
    //climber.updateShuffleboard();
    //collector.updateShuffleboard();
    //limelight.updateShuffleBoard();
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
            .setKinematics(Constants.kDriveKinematics);

    tuningTrajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
      new Pose2d(),
      new Pose2d(3, 0, new Rotation2d())
    ), 
    config);

    try {
      manualTrajectory1 = generateTrajectoryFromJSON(manualPath1);
      fourBallAutoTrajectory1 = generateTrajectoryFromJSON(fourBallAutoPath1);
      fourBallAutoTrajectory2 = generateTrajectoryFromJSON(fourBallAutoPath2);
      fourBallAutoTrajectory3 = generateTrajectoryFromJSON(fourBallAutoPath3);
      fourBallAutoTrajectory4 = generateTrajectoryFromJSON(fourBallAutoPath4);
      threeBallAutoTrajectory3 = generateTrajectoryFromJSON(threeBallAutoPath3);
      gatekeepPathTrajectory1 = generateTrajectoryFromJSON(gatewayPath1);
      gatekeepPathTrajectory2 = generateTrajectoryFromJSON(gatewayPath2);
      gatekeepPathTrajectory3 = generateTrajectoryFromJSON(gatewayPath3);
      fiveBallAutoTrajectory1 = generateTrajectoryFromJSON(fiveBallAutoPath1);
      fiveBallAutoTrajectory2 = generateTrajectoryFromJSON(fiveBallAutoPath2);
      fiveBallAutoTrajectory3 = generateTrajectoryFromJSON(fiveBallAutoPath3);
      tuningTrajectory = generateTrajectoryFromJSON(tuningPath);

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

  public Drive getDrive() {
    return drive;
  }

  public XboxController getDriveStick(){
    return driveStick;
  }

  public XboxController getOpStick(){
    return opStick;
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
