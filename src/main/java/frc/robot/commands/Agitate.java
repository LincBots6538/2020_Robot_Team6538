/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Agitator;

import static frc.robot.Constants.*;

public class Agitate extends CommandBase {
  /**
   * Creates a new Agitate.
   */
  private Agitator agitator_subsystem;
  private boolean reverse;

  public Agitate(Agitator subsystem, boolean _reverse) {
    agitator_subsystem = subsystem;
    reverse = _reverse;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(reverse)
    {
      agitator_subsystem.setVelocity(-AGITATOR_SPEED);
    } else
    {
      agitator_subsystem.setVelocity(AGITATOR_SPEED);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    agitator_subsystem.stopAgitator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
