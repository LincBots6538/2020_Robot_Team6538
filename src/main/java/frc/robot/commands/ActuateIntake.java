/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import static frc.robot.Constants.*;

public class ActuateIntake extends CommandBase {
  private Intake intake_subsystem;
  /**
   * Creates a new ActuateIntake.
   */
  public ActuateIntake(Intake subsystem) {
    intake_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake_subsystem.set(INTAKE_WHEEL_SPEED);
    intake_subsystem.setSolenoid(Value.kForward);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake_subsystem.setSolenoid(Value.kReverse);
    intake_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
