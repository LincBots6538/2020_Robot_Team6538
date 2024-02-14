/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.*;

public class ManualSpinIntakeRollers extends CommandBase {
  /**
   * Creates a new ManualSpinIntakeRollers.
   */

  Intake intake_subsystem;
  boolean reverse;
  
  public ManualSpinIntakeRollers(Intake subsystem, boolean _reverse) {
    intake_subsystem = subsystem;
    reverse = _reverse;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(reverse)
    {
      intake_subsystem.set(-INTAKE_WHEEL_SPEED);
    }
    else
    {
      intake_subsystem.set(INTAKE_WHEEL_SPEED);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
