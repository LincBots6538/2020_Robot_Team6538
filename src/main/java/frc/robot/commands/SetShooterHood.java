/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetShooterHood extends InstantCommand {
  Shooter shooter_subsystem;
  boolean state;

  public SetShooterHood(Shooter subsystem, boolean _state) {
    shooter_subsystem = subsystem;
    state = _state;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(state)
    {
      shooter_subsystem.setSolenoid(true);
    } else
    {
      shooter_subsystem.setSolenoid(false);
    }
  }
}
