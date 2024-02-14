/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SetShooterVelocity extends CommandBase {
  /**
   * Creates a new SetShooterVelocity.
   * Command to set the flywheel of the shooter to a desired velocity.
   */

  private Shooter shooter_subsystem;
  private int velocity;

  public SetShooterVelocity(Shooter subsystem, int _velocity) {
    // Pass in and require the shooter subsystem
    shooter_subsystem = subsystem;
    velocity = _velocity;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //shooter_subsystem.setVelocity(velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter_subsystem.stop();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
