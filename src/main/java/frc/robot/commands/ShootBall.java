/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootBall extends SequentialCommandGroup {
  /**
   * Creates a new ShootBall.
   * First, the Limelight should have already automatically aligned the turret to the vision target
   * and sped the shooter up to an appropriate velocity.
   */
  public ShootBall(Agitator agitator_subsystem, boolean _reverse,
                    Conveyor conveyor_subsystem,
                    Shooter shooter_subsystem, int shooter_velocity) {
    super(new SetShooterVelocity(shooter_subsystem, shooter_velocity),
          new Agitate(agitator_subsystem, _reverse),
          new ActivateConveyor(conveyor_subsystem));
  }
}

