/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonSRX m_wheel;
  private DoubleSolenoid solenoid;

  /**
   * Creates a new Intake.
   */
  public Intake(int motorID, boolean invertHBridge, int PCM_ID, int forwardDoubleSolenoidChannel, int reverseDoubleSolenoidChannel) {
    m_wheel = new TalonSRX(motorID);
    m_wheel.configFactoryDefault();
    solenoid = new DoubleSolenoid(PCM_ID, forwardDoubleSolenoidChannel, reverseDoubleSolenoidChannel);
    if(invertHBridge) { m_wheel.setInverted(true); }
  }

  public void set(double velocity) {
    m_wheel.set(ControlMode.PercentOutput, velocity);
  }

  public void stop() {
    m_wheel.set(ControlMode.PercentOutput, 0.0);
  }

  public void setSolenoid(DoubleSolenoid.Value value) {
    solenoid.set(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
