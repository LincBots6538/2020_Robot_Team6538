/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {

  /**
   * Differential drivetrain is composed of a master and slave Falcon 500
   * on the left and right sides of the robot.
   */
  private TalonFX m_masterleft, m_slaveleft;
  private TalonFX m_masterright, m_slaveright;

  private TalonFXConfiguration drivetrain_configs;

  /**
   * Create a new Tab on the Shuffleboard for drivetrain information.
   */
  private ShuffleboardTab tab_drivetrain = Shuffleboard.getTab("Drivetrain");
  private NetworkTableEntry leftEncMeasureEntry = tab_drivetrain.add("Left Encoder Measurement (inches)", 0).getEntry();
  private NetworkTableEntry rightEncMeasureEntry = tab_drivetrain.add("Right Encoder Measurement (inches)", 0).getEntry();
  private NetworkTableEntry leftTargetEntry = tab_drivetrain.add("Left-side Target", 0).getEntry();
  private NetworkTableEntry rightTargetEntry = tab_drivetrain.add("Right-side Target", 0).getEntry();
  private NetworkTableEntry leftVelocityEntry = tab_drivetrain.add("Left-side Velocity", 0).getEntry();
  private NetworkTableEntry rightVelocityEntry = tab_drivetrain.add("Right-side Velocity", 0).getEntry();

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    m_masterleft = new TalonFX(DRIVETRAIN_FRONTLEFT_MOTOR);
    m_slaveleft = new TalonFX(DRIVETRAIN_REARLEFT_MOTOR);
    m_masterright = new TalonFX(DRIVETRAIN_FRONTRIGHT_MOTOR);
    m_slaveright = new TalonFX(DRIVETRAIN_REARRIGHT_MOTOR);

    m_masterleft.configFactoryDefault();
    m_slaveleft.configFactoryDefault();
    m_masterright.configFactoryDefault();
    m_slaveright.configFactoryDefault();

    drivetrain_configs = new TalonFXConfiguration();

    // Configure the 'master' motor controllers on each side to read from their attached encoders
    drivetrain_configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    m_masterleft.configAllSettings(drivetrain_configs);
    m_masterright.configAllSettings(drivetrain_configs);

    // Invert the right-side motor controllers so that positive input will result in forward motion
    m_masterright.setInverted(true);
    m_slaveright.setInverted(true);

    // Set the PID gains of the 'master' motor controllers
    m_masterleft.config_kP(SHOOTER_kPIDLoopIdx, DRIVETRAIN_kGains.kP);
    m_masterleft.config_kI(SHOOTER_kPIDLoopIdx, DRIVETRAIN_kGains.kI);
    m_masterleft.config_kD(SHOOTER_kPIDLoopIdx, DRIVETRAIN_kGains.kD);

    m_masterright.config_kP(SHOOTER_kPIDLoopIdx, DRIVETRAIN_kGains.kP);
    m_masterright.config_kI(SHOOTER_kPIDLoopIdx, DRIVETRAIN_kGains.kI);
    m_masterright.config_kD(SHOOTER_kPIDLoopIdx, DRIVETRAIN_kGains.kD);

    /**
     * Control the Acceleration/Deceleration of the drivetrain through the desired
     * time it takes to go from 0 to full throttle.
     */
    m_masterleft.configOpenloopRamp(DRIVETRAIN_RAMP_TIME);
    m_masterleft.configClosedloopRamp(DRIVETRAIN_RAMP_TIME);

    m_masterright.configOpenloopRamp(DRIVETRAIN_RAMP_TIME);
    m_masterright.configClosedloopRamp(DRIVETRAIN_RAMP_TIME);

    //m_masterleft.configAllowableClosedloopError(0, convertToRawUnits(DRIVETRAIN_CLOSED_LOOP_TOLERANCE));
    //m_masterright.configAllowableClosedloopError(0, convertToRawUnits(DRIVETRAIN_CLOSED_LOOP_TOLERANCE));

    // Set the 'slave' motor controllers to receice the same amount of current as their respective 'masters'
    m_slaveleft.follow(m_masterleft);
    m_slaveright.follow(m_masterright);
  }

  /**
   * Functions to either convert from encoder sensor units to the tangential distance
   * the drivetrain wheels have moved in inches, or the inverse (inches to sensor units).
   */
  public double convertToInches(double _raw_units) {
    double raw_units = _raw_units; // Raw sensor units
    // Convert raw units into rotations
    double rotations = raw_units / DRIVETRAIN_kUnitsPerRevolution;
    // Convert rotations into tangential inches
    double inches = (rotations * DRIVETRAIN_WHEEL_CIRCUMFERENCE) / DRIVETRAIN_GEAR_RATIO;
    return inches;
  }

  public int convertToRawUnits(double _inches) {
    double inches = _inches; // units in inches
    // Convert inches into raw unit rotations
    double rotations = (inches * DRIVETRAIN_GEAR_RATIO) / DRIVETRAIN_WHEEL_CIRCUMFERENCE;
    // Convert rotations into raw units
    int raw_units = (int) (rotations * DRIVETRAIN_kUnitsPerRevolution);
    return raw_units;
  }

  // Returns wheel distance traveled in inches
  public double getLeftEncoderMeasurement() {
    return convertToInches(m_masterleft.getSelectedSensorPosition());
  }
/*
  // Returns closed-loop target
  public double getLeftTarget() {
    return m_masterleft.getClosedLoopTarget();
  }*/
/*
  // Returns closed-loop target in inches
  public double getLeftTargetMeasurement() {
    return convertToInches(m_masterleft.getClosedLoopTarget());
  }*/

  // Returns velocity in sensor units per 100ms
  public double getLeftVelocity() {
    return m_masterleft.getSelectedSensorVelocity();
  }

  // Returns wheel distance traveled in inches
  public double getRightEncoderMeasurement() {
    return convertToInches(m_masterright.getSelectedSensorPosition());
  }
/*
  // Returns closed-loo target
  public double getRightTarget() {
    return m_masterright.getClosedLoopTarget();
  }*/
/*
  // Returns closed-loop target in inches
  public double getRightTargetMeasurement() {
    return convertToInches(m_masterright.getClosedLoopTarget());
  }*/

  // Returns velocity in sensor units per 100ms
  public double getRightVelocity() {
    return m_masterright.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Stops the drivetrain based around a position to set the robot at
  public void PIDStop() {
    double leftEncoderPos = getLeftEncoderMeasurement();
    double rightEncoderPos = getRightEncoderMeasurement();

    setLeftTarget(leftEncoderPos);
    setRightTarget(rightEncoderPos);
  }

  // Updates the values on the Shuffleboard
  public void putValuesToShuffleboard() {
    // Encoder measurements in inches
    leftEncMeasureEntry.setDouble(getLeftEncoderMeasurement());
    rightEncMeasureEntry.setDouble(getRightEncoderMeasurement());
    
    // Velocities in raw sensor units per 100ms
    leftVelocityEntry.setDouble(getLeftVelocity());
    rightVelocityEntry.setDouble(getRightVelocity());
    //leftVelocityEntry.setDouble(Robot.m_robotContainer.getController(0).getX(Hand.kLeft));
    //rightVelocityEntry.setDouble(Robot.m_robotContainer.getController(0).getRawAxis(3));
/*
    // Closed-loop targets
    leftTargetEntry.setDouble(getLeftTarget());
    rightTargetEntry.setDouble(getRightTarget());*/
  }

  // Resets the drivetrain encoders back to 0 units
  public void resetEncoders() {
    m_masterleft.setSelectedSensorPosition(0);
    m_masterright.setSelectedSensorPosition(0);
  }

  // Set the closed-loop distance target
  public void setLeftTarget(double inches) {
    m_masterleft.set(ControlMode.Position, convertToRawUnits(inches));
  }

  // Set the closed-loop velocity target
  public void setLeftVelocity(double velocity) {
    m_masterleft.set(ControlMode.Velocity, velocity);
  }

  // Set the closd-loop distance target
  public void setRightTarget(double inches) {
    m_masterright.set(ControlMode.Position, convertToRawUnits(inches));
  }

  // Set the closed-loop velocity target
  public void setRightVelocity(double velocity) {
    m_masterright.set(ControlMode.Velocity, velocity);
  }

  // Stop the drivetrain based on a closed-loop velocity target of zero units per 100ms
  public void stop() {
    m_masterleft.set(ControlMode.Velocity, 0.0);
    m_masterright.set(ControlMode.Velocity, 0.0);
  }
}
