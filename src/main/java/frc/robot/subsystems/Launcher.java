// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  public enum ShooterPosition {
    LOWER_HUB(0, "Lower Hub"), UPPER_HUB(1, "Upper Hub"), TARMAC_LOW(2, "Tarmac Low"), TARMAC_HIGH(3, "Tarmac High"), HALF_COURT(4, "Half Court"),  DEAD_ZONE(5, "Dead Zone"),  INVALID(6, "Invalid");

    private int value;
    private String name;
    
    private ShooterPosition(int value, String name) {
      this.value = value;
      this.name = name;
    }

    public int getPosition() {
      return value;
    }

    public String getName() {
      return name;
    }

    public static ShooterPosition fromInt(int value) {
      for(ShooterPosition position : values()) {
        if(position.value == value) {
          return position;
        }
      }
      return INVALID;
    }

    public static int getHighestValue() {
      int highestVal = 0;
      for(ShooterPosition position : values()) {
        if(position.value > highestVal) {
          highestVal = position.value;
        }
      }
      return highestVal - 2;
    }
  }

  public enum Direction {
    kForwards(1), kBackwards(-1);

    private int direction;

    private Direction(int direction) {
      this.direction = direction;
    }

    public int getDirection() {
      return direction;
    }
  }

  private WPI_TalonFX bottomRoller;
  private WPI_TalonFX topRoller;

  private boolean pidEnabled = false;

  private boolean overrideLimelight = false;

  private static final double[] TOPROLLER_SETPOINT = {
    14000, 
    16000, 
    12500,
    11000,
    12000,
    15000
  };
  private static final double[] BOTTOMROLLER_SETPOINT = {
    5000, 
    5000, 
    9000,
    9500,
    1000,
    7500
  };
  private static final double[] TOPROLLER_P = {
    0.01, 
    0.01, 
    0.01, 
    0.01,
    0.01,
    0.01
  };
  private static final double[] TOPROLLER_F = {
    (0.30 * 1023.0 / TOPROLLER_SETPOINT[0]), 
    ((TOPROLLER_SETPOINT[1] / 20800) * 1023.0 / TOPROLLER_SETPOINT[1]), 
    ((TOPROLLER_SETPOINT[2] / 20800) * 1023.0 / TOPROLLER_SETPOINT[2]), 
    ((TOPROLLER_SETPOINT[3] / 20800) * 1023.0 / TOPROLLER_SETPOINT[3]),
    ((TOPROLLER_SETPOINT[4] / 20800) * 1023.0 / TOPROLLER_SETPOINT[4]),
    ((TOPROLLER_SETPOINT[5] / 20800) * 1023.0 / TOPROLLER_SETPOINT[5]),



  };
  private static final double[] BOTTOMROLLER_P = {
    0.01, 
    0.01, 
    0.01, 
    0.0075,
    0.01,
    0.01
  };
  private static final double[] BOTTOMROLLER_F = {
    (0.23 * 1023.0 / BOTTOMROLLER_SETPOINT[0]), 
    ((BOTTOMROLLER_SETPOINT[1] / 20800) * 1023.0 / BOTTOMROLLER_SETPOINT[1]), 
    ((BOTTOMROLLER_SETPOINT[2] / 20800) * 1023.0 / BOTTOMROLLER_SETPOINT[2]), 
    ((BOTTOMROLLER_SETPOINT[3] / 20800) * 1023.0 / BOTTOMROLLER_SETPOINT[3]),
    ((BOTTOMROLLER_SETPOINT[4] / 20800) * 1023.0 / BOTTOMROLLER_SETPOINT[4]),
    ((BOTTOMROLLER_SETPOINT[5] / 20800) * 1023.0 / BOTTOMROLLER_SETPOINT[5]),
  };
  
  //Setpoint Values: 3400, 4100, 4500

  private double pBottom, fBottom, setpointBottom,
                 pTop, fTop, setpointTop;

  private ShooterPosition defaultPosition = ShooterPosition.LOWER_HUB;
  private ShooterPosition selectedPosition = defaultPosition;

  private static final double MAX_OUTPUT = 1;
  private static final double MIN_OUTPUT = -1;


  
  /** Creates a new Shooter. */
  public Launcher() {
    
    bottomRoller = new WPI_TalonFX(21);
    topRoller = new WPI_TalonFX(22);

    config();
    setGainPreset(defaultPosition);
  }

  public void config() {
    bottomRoller.configFactoryDefault();
    topRoller.configFactoryDefault();

    bottomRoller.setNeutralMode(NeutralMode.Coast);
    topRoller.setNeutralMode(NeutralMode.Coast);

    bottomRoller.setInverted(true);
    topRoller.setInverted(true);

    bottomRoller.configNeutralDeadband(0.001);
    topRoller.configNeutralDeadband(0.001);

    bottomRoller.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    topRoller.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);

    bottomRoller.configPeakOutputForward(MAX_OUTPUT);
    bottomRoller.configPeakOutputReverse(MIN_OUTPUT);
    topRoller.configPeakOutputForward(MAX_OUTPUT);
    topRoller.configPeakOutputReverse(MIN_OUTPUT);

    bottomRoller.configNominalOutputForward(0);
    bottomRoller.configNominalOutputReverse(0);
    topRoller.configNominalOutputForward(0);
    topRoller.configNominalOutputReverse(0);

    bottomRoller.config_kP(0, BOTTOMROLLER_P[selectedPosition.getPosition()], 100);
    bottomRoller.config_kF(0, BOTTOMROLLER_F[selectedPosition.getPosition()], 100);
    topRoller.config_kP(0, TOPROLLER_P[selectedPosition.getPosition()], 100);
    topRoller.config_kF(0, TOPROLLER_F[selectedPosition.getPosition()], 100);
  }

  public void updateShuffleboard() {
    // SmartDashboard.putNumber("Bottom Shooter Velocity", bottomRoller.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Bottom Shooter Power", bottomRoller.get());
    // SmartDashboard.putNumber("Bottom Shooter Value", bottomRoller.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Top Shooter Velocity", topRoller.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Top Shooter Power", topRoller.get());
    // SmartDashboard.putNumber("Top Shooter Value", topRoller.getSelectedSensorPosition());
    
    SmartDashboard.putString("Launcher Position", selectedPosition.getName());
    SmartDashboard.putBoolean("Launcher Override", getOverride());

    /*double pBottom = SmartDashboard.getNumber("Bottom Launcher P", this.pBottom);
    double fBottom = SmartDashboard.getNumber("Bottom Launcher F", this.fBottom);
    double setpointBottom = SmartDashboard.getNumber("Bottom Launcher Setpoint", this.setpointBottom);

    double pTop = SmartDashboard.getNumber("Top Launcher P", this.pTop);
    double fTop = SmartDashboard.getNumber("Top Launcher F", this.fTop);
    double setpointTop = SmartDashboard.getNumber("Top Launcher Setpoint", this.setpointTop);

    // If there are any changes from Shuffleboard, update the PID Controller
    if (this.pBottom != pBottom || this.fBottom != fBottom || this.setpointBottom != setpointBottom) {
      setBottomPIDGains(pBottom, fBottom, setpointBottom);
      System.out.println("Updating Values");
    }

    if (this.pTop != pTop || this.fTop != fTop || this.setpointTop != setpointTop) {
      setTopPIDGains(pTop, fTop, setpointTop);
      System.out.println("Updating Values");
    }*/
  }

  public void setLauncherTop(double speed){
    topRoller.set(TalonFXControlMode.PercentOutput, -speed);
  }

  public void setLauncherBottom(double speed) {
    bottomRoller.set(TalonFXControlMode.PercentOutput, -speed);
  }

  public void setLauncher(double bottomRollerSpeed, double topRollerSpeed) {
    setLauncherTop(topRollerSpeed);
    setLauncherBottom(bottomRollerSpeed);
  }
  
  public void setLauncherForPosition() {
    if(selectedPosition.equals(ShooterPosition.LOWER_HUB)) {
      setLauncherTop(0.23);
      setLauncherBottom(0.23);
    }
    if(selectedPosition.equals(ShooterPosition.UPPER_HUB)) {
      setLauncherTop(0.47);
      setLauncherBottom(0.47);
    }
  }

  public void stop() {
    topRoller.set(0);
    bottomRoller.set(0);
  }

  public void resetRightEncoders(){
    topRoller.setSelectedSensorPosition(0, 0, 100);  
  }

  public void resetLeftEncoders(){
    bottomRoller.setSelectedSensorPosition(0, 0, 100);
  }

  public double getRightEncoder(){
   return topRoller.getSelectedSensorPosition();
  }
  
  public double getLeftEncoder(){
    return bottomRoller.getSelectedSensorPosition();
  }

  /**
   * Convenience wrapper that sets PID parameters according to pre-stored values that
   * correspond with the given shooter position
   * @param shooterPosition Indicates the shooter position for which PID values will be updated
   */
  public void setGainPreset(ShooterPosition shooterPosition) {
    int pos = shooterPosition.getPosition();
    selectedPosition = shooterPosition;

    // I and D values are not pulled from an array since these values are always zero
    setTopPIDGains(TOPROLLER_P[pos], TOPROLLER_F[pos], TOPROLLER_SETPOINT[pos]);
    setBottomPIDGains(BOTTOMROLLER_P[pos], BOTTOMROLLER_F[pos], BOTTOMROLLER_SETPOINT[pos]);
  }

  /**
   * Sets all tuning parameters for the rotation-regulating PID
   * @param p P gain
   * @param i I gain
   * @param d D gain
   * @param f Feed-forward gain
   * @param setpoint RPM setpoint
   */
  public void setBottomPIDGains(double p, double f, double setpoint) {
    this.pBottom = p;
    this.fBottom = f;
    this.setpointBottom = setpoint;

    bottomRoller.config_kP(0, p);
    bottomRoller.config_kF(0, f);

    /*SmartDashboard.putNumber("Bottom Launcher P", p);
    SmartDashboard.putNumber("Bottom Launcher F", f);
    SmartDashboard.putNumber("Bottom Launcher Setpoint", setpoint);*/
  }

  public void setTopPIDGains(double p, double f, double setpoint) {
    this.pTop = p;
    this.fTop = f;
    this.setpointTop = setpoint;

    topRoller.config_kP(0, p);
    topRoller.config_kF(0, f);

    /*SmartDashboard.putNumber("Top Launcher P", p);
    SmartDashboard.putNumber("Top Launcher F", f);
    SmartDashboard.putNumber("Top Launcher Setpoint", setpoint);*/
  }

  /**
   * Convenience method that sets PID parameters according to pre-stored default values
   */
  public void resetGainPreset() {
    setGainPreset(defaultPosition);
  }

  /**
   * Sets a flag that will signal the PID to run periodically
   */
  public void pidOn() {
    if (!pidEnabled) {
      pidEnabled = true;
      bottomRoller.setIntegralAccumulator(0, 0, 100);
      topRoller.setIntegralAccumulator(0, 0, 100);
    }
  }

  /**
   * Runs all periodic logic for the PID controller, called periodically while
   * the PID is enabled
   */
  private void pidPeriodic() {
    bottomRoller.set(TalonFXControlMode.Velocity, -setpointBottom);
    topRoller.set(TalonFXControlMode.Velocity, -setpointTop);
  }

  public void cycleGainPreset(Direction direction) {
    int highestVal = ShooterPosition.getHighestValue();
    int nextPosition = selectedPosition.getPosition() + direction.getDirection();
    if(nextPosition < 0) {
      selectedPosition = ShooterPosition.fromInt(highestVal);
    } else if(nextPosition > highestVal) {
      selectedPosition = ShooterPosition.fromInt(0);
    } else {
      selectedPosition = ShooterPosition.fromInt(nextPosition);
    }
    setGainPreset(selectedPosition);
  }

  public double getShooterVelocity() {
    return bottomRoller.getSelectedSensorVelocity();
  }

  public double getTopSetPoint() {
    return TOPROLLER_SETPOINT[selectedPosition.getPosition()];
  }
  public double getBottomSetPoint() {
    return BOTTOMROLLER_SETPOINT[selectedPosition.getPosition()];
  }

  public boolean isAtTopSetpoint() {
    return topRoller.getSelectedSensorVelocity() >= getTopSetPoint() - 50 || topRoller.getSelectedSensorVelocity() <= getTopSetPoint() + 50;
  }

  public boolean isAtBottomSetpoint() {
    return bottomRoller.getSelectedSensorVelocity() >= getBottomSetPoint() - 50 || bottomRoller.getSelectedSensorVelocity() <= getBottomSetPoint() + 50;
  }

  public ShooterPosition getSelectedPosition() {
    return selectedPosition;
  }

  public void toggleLimelightOverride() {
    overrideLimelight = overrideLimelight ? false : true;
  }

  public boolean getOverride() {
    return overrideLimelight;
  }

  /**
   * Sets a flag that will prevent the PID from running periodically
   */
  public void pidOff() {
    pidEnabled = false;
    stop();
  }

  @Override
  public void periodic() {
    if (pidEnabled) {
      pidPeriodic();
    }
  }
}