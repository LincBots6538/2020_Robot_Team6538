/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Agitator extends SubsystemBase {
  /**
   * Creates a new Agitator.
   */

  private TalonSRX m_talon;

  public Agitator() {
    m_talon = new TalonSRX(AGITATOR_MOTOR);
    m_talon.configFactoryDefault();
  }

  public void setVelocity(double velocity) {
    m_talon.set(ControlMode.PercentOutput, velocity);
  }

  public void stopAgitator() {
    m_talon.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
