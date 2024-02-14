/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretLEDController;

import static frc.robot.Constants.*;

public class BlinkLEDStripColor extends CommandBase {
  /**
   * Creates a new BlinkLEDStripColor.
   */

  TurretLEDController blinkin;
  double color;
  Timer timer;
  boolean state;

  public BlinkLEDStripColor(TurretLEDController _blinkin, double colorValue) {
    blinkin = _blinkin;
    color = colorValue;
    timer = new Timer();
    
    addRequirements(_blinkin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    blinkin.setColor(color);
    state = true;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ms is the time in milliseconds
    double ms = timer.get() * 1000;
    // every 250ms change the state of the blink (either to 'on' or 'off')
    if( ms % 250 == 0 )
    {
      state = !state;
    }

    // Depending on the state, turn the LEDs 'on' or 'off'
    if(state)
    {
      blinkin.setColor(color);
    }
    else
    {
      blinkin.setColor(LED_COLOR_OFF);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    blinkin.setColor(LED_COLOR_OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
