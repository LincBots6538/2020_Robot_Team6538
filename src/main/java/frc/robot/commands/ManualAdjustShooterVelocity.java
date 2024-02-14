/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ManualAdjustShooterVelocity extends InstantCommand {

  private Shooter shooter_subsystem;
  private boolean direction;
  private double current_percent_output, new_percent_output;

  public ManualAdjustShooterVelocity(Shooter _subsystem, boolean _direction) {
    shooter_subsystem = _subsystem;
    direction = _direction;
    addRequirements(_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get the current percent-output of the shooter
    current_percent_output = shooter_subsystem.getPercentOutput();

    // If increasing the velocity (direction == true --> forward direction)
    if(direction)
    {
      // Set the new percent-output to be an increase of 5%
      new_percent_output = current_percent_output + 0.05;
    }
    else // If decreasing the velocity (direction == false --> reverse direction)
    {
      // Set the new percent-output to be a decrease of 5%
      new_percent_output = current_percent_output - 0.05;
    }

    // Set the shooter motors to the new percent-output, within the boundaries of -100% and 100%
    shooter_subsystem.set(MathUtil.clamp(new_percent_output, -1.0, 1.0));
  }
}
