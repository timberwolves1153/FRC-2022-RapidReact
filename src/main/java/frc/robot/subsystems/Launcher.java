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
import frc.robot.lib.Direction;
import frc.robot.lib.ShooterPosition;

public class Launcher extends SubsystemBase {
  private WPI_TalonFX bottomRoller;
  private WPI_TalonFX topRoller;

  private boolean pidEnabled = false;

  private boolean overrideLimelight = false;
  
  //Setpoint Values: 3400, 4100, 4500

  private double pBottom, fBottom, setpointBottom,
                 pTop, fTop, setpointTop;

  private ShooterPosition defaultPosition = ShooterPosition.FENDER_HIGH;
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

    bottomRoller.config_kP(0, selectedPosition.getBottomGains().getP(), 100);
    bottomRoller.config_kF(0, selectedPosition.getBottomGains().getF(), 100);
    topRoller.config_kP(0, selectedPosition.getTopGains().getP(), 100);
    topRoller.config_kF(0, selectedPosition.getTopGains().getF(), 100);
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

    double pBottom = SmartDashboard.getNumber("Bottom Launcher P", this.pBottom);
    double fBottom = SmartDashboard.getNumber("Bottom Launcher F", this.fBottom);
    double setpointBottom = SmartDashboard.getNumber("Bottom Launcher Setpoint", this.setpointBottom);

    double pTop = SmartDashboard.getNumber("Top Launcher P", this.pTop);
    double fTop = SmartDashboard.getNumber("Top Launcher F", this.fTop);
    double setpointTop = SmartDashboard.getNumber("Top Launcher Setpoint", this.setpointTop);

    //If there are any changes from Shuffleboard, update the PID Controller
    if (this.pBottom != pBottom || this.fBottom != fBottom || this.setpointBottom != setpointBottom) {
      setBottomPIDGains(pBottom, fBottom, setpointBottom);
      System.out.println("Updating Values");
    }

    if (this.pTop != pTop || this.fTop != fTop || this.setpointTop != setpointTop) {
      setTopPIDGains(pTop, fTop, setpointTop);
      System.out.println("Updating Values");
    }
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
    if(selectedPosition.equals(ShooterPosition.FENDER_LOW)) {
      setLauncherTop(0.23);
      setLauncherBottom(0.23);
    }
    if(selectedPosition.equals(ShooterPosition.FENDER_HIGH)) {
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
    selectedPosition = shooterPosition;
    
    setTopPIDGains(shooterPosition.getTopGains().getP(), shooterPosition.getTopGains().getF(), shooterPosition.getTopSetpoint());
    setBottomPIDGains(shooterPosition.getBottomGains().getP(), shooterPosition.getBottomGains().getF(), shooterPosition.getBottomSetpoint());
  }

  /**
   * Sets all tuning parameters for the rotation-regulating PID
   * @param p P gain
   * @param f Feed-forward gain
   * @param setpoint RPM setpoint
   */
  public void setBottomPIDGains(double p, double f, double setpoint) {
    this.pBottom = p;
    this.fBottom = f;
    this.setpointBottom = setpoint;

    bottomRoller.config_kP(0, p);
    bottomRoller.config_kF(0, f);

    SmartDashboard.putNumber("Bottom Launcher P", p);
    SmartDashboard.putNumber("Bottom Launcher F", f);
    SmartDashboard.putNumber("Bottom Launcher Setpoint", setpoint);
  }

  public void setTopPIDGains(double p, double f, double setpoint) {
    this.pTop = p;
    this.fTop = f;
    this.setpointTop = setpoint;

    topRoller.config_kP(0, p);
    topRoller.config_kF(0, f);

    SmartDashboard.putNumber("Top Launcher P", p);
    SmartDashboard.putNumber("Top Launcher F", f);
    SmartDashboard.putNumber("Top Launcher Setpoint", setpoint);
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
    int highestVal = ShooterPosition.getHighestIndex();
    int nextPosition = selectedPosition.getIndex() + direction.getDirection();
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
    return selectedPosition.getTopSetpoint();
  }
  public double getBottomSetPoint() {
    return selectedPosition.getBottomSetpoint();
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