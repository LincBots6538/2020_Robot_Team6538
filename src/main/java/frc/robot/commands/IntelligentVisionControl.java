/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretLEDController;

import static frc.robot.Constants.*;

public class IntelligentVisionControl extends CommandBase {
  /**
   * Creates a new IntelligentVisionControl.
   */
  private boolean autonomous;
  private Turret turret_subsystem;
  private Shooter shooter_subsystem;
  private Conveyor conveyor_subsystem;
  private Limelight limelight;
  private TurretLEDController blinkin;
  private double tx;
  private double pidOutput;
  private double distance_to_vision_target;
  private int desired_velocity;
  private Command setRedLEDsCommand, blinkOrangeLEDsCommand, setGreenLEDsCommand;

  public IntelligentVisionControl(boolean _autonomous, Turret turret_subsystem, Shooter shooter_subsystem, Conveyor conveyor_subsystem, Limelight limelight, TurretLEDController _blinkin) {
    autonomous = _autonomous;
    this.turret_subsystem = turret_subsystem;
    this.shooter_subsystem = shooter_subsystem;
    this.conveyor_subsystem = conveyor_subsystem;
    this.limelight = limelight;
    blinkin = _blinkin;

    setRedLEDsCommand = new SetLEDStripColor(_blinkin, LED_COLOR_RED);
    blinkOrangeLEDsCommand = new BlinkLEDStripColor(_blinkin, LED_COLOR_ORANGE);
    setGreenLEDsCommand = new SetLEDStripColor(_blinkin, LED_COLOR_GREEN);

    addRequirements(turret_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /**
     * Turn on the Limelight's LEDs and set to to its vision processing mode
     */
    limelight.setLedMode(3);
    limelight.setCameraMode(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If the Limelight can detect a vision target
    if(limelight.foundTarget())
    {
      // horizontal angle of error from the center of the vision target (in degrees)
      tx = limelight.get_tx();

      /**
       * Control the LED strip color.
       * If the Limelight is not within a certain angle tolerance toward the vision target, make
       * the LEDs blink an orange color. If the Limelight is within the tolerance, make the LEDs a green color.
       */
      if(Math.abs(tx) > VISION_ANGLE_TOLERANCE)
      {
        // Set LEDs to an orange color
        blinkOrangeLEDsCommand.schedule();
      }
      else
      {
        // Set LEDs to a green color
        setGreenLEDsCommand.schedule();
      }

      /**
       * Control the turret to automatically center itself toward the vision target.
       */
      turret_subsystem.setPIDSetpoint(0.0);
      pidOutput = turret_subsystem.getPIDOutput(tx);

      turret_subsystem.set(pidOutput);

      distance_to_vision_target = limelight.getDistanceToTarget(VISION_HEIGHT_OF_POWER_PORT_TARGET);

      /**
       * Control the shooter hood to automatically switch between its active and inactive positions
       * based on the distance the robot is away from a vision target.
       */
      if(distance_to_vision_target >= VISION_SHOOTER_HOOD_SWITCH_DISTANCE)
      {
        shooter_subsystem.setSolenoid(true);
      } else
      {
        shooter_subsystem.setSolenoid(false);
      }

      /**
       * Control the shooter velocity automatically based on how far the robot is from the vision target.
       */
      desired_velocity = getDesiredShooterVelocity(distance_to_vision_target);
      if(autonomous)
      {
        shooter_subsystem.set(desired_velocity/SHOOTER_PEAK_SPEED);
        /*
        if(Math.abs(tx) <= VISION_ANGLE_TOLERANCE
          && shooter_subsystem.withinVelocityTolerance(desired_velocity))
        {
          conveyor_subsystem.set(CONVEYOR_SPEED);
          Timer.delay(1);
          end(true);
        }
        */
        Timer.delay(1);
        turret_subsystem.stop();
        conveyor_subsystem.set(CONVEYOR_SPEED);
        Timer.delay(2);
        conveyor_subsystem.stop();
        end(true);
      }
      else
      {
        shooter_subsystem.set(desired_velocity/SHOOTER_PEAK_SPEED);
      }
    }
    else{ // If the Limelight cannot detect a vision target
      // Set LEDs to a red color
      setRedLEDsCommand.schedule();
      // Stop the turret's movement
      turret_subsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret_subsystem.stop();
    //shooter_subsystem.stop();
    /**
     * Turn off the Limelight's LEDs and set to to its 'driver camera' mode
     */
    limelight.setLedMode(1);
    limelight.setCameraMode(true);
    // Turn off the turret's LEDs
    blinkin.turnOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private int getDesiredShooterVelocity(double distance)
  {
    return (int) ( (Math.pow(distance, 2) * 0.00000109 + 0.524) * SHOOTER_PEAK_SPEED );
    //return (int) ((0.00039 * distance + 0.492) * SHOOTER_PEAK_SPEED);
    /*
    if(distance < 121)
    {
      return (int) (SHOOTER_PEAK_SPEED * 0.54);
    }
    else if(distance < 210)
    {
      return (int) (SHOOTER_PEAK_SPEED * 0.54);
    }
    else {
      return (int) (SHOOTER_PEAK_SPEED * 0.59);
    }
    */
  }
}
