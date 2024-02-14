/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;

public class VelocityPIDTeleopDrive extends CommandBase {
  /**
   * Creates a new PIDTeleopDrive.
   */
  private Drivetrain drivetrain_subsystem;

  private XboxController controller;
  private double leftTriggerVal, rightTriggerVal, stickVal;
  
  private double leftVelocity, rightVelocity;
  
  public VelocityPIDTeleopDrive(Drivetrain subsystem) {
    // Pass the drivetrain Subsystem into the Command and have the Command require it to run
    drivetrain_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Pass the controller on port 0 into the Command
    controller = (XboxController) Robot.m_robotContainer.getController(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Xbox contoller left trigger multiplied by -1 to control reverse motion
    leftTriggerVal = controller.getTriggerAxis(Hand.kLeft) * -1;
    // Xbox controller right trigger to control forward motion
    rightTriggerVal = controller.getTriggerAxis(Hand.kRight);
    // Xbox controller left stick (x-axis) to controller turning
    stickVal = controller.getX(Hand.kLeft);

    // Create deadbands for the input triggers and stick. If input is less than 10%, read no input.
    if(Math.abs(leftTriggerVal) < 0.1) { leftTriggerVal = 0.0; }
    if(Math.abs(rightTriggerVal) < 0.1) { rightTriggerVal = 0.0; }
    if(Math.abs(stickVal) < 0.1) { stickVal = 0.0; }

    /**
     * Square all the inputs to make low-speed handling smoother.
     */
    leftTriggerVal = Math.copySign(Math.pow(leftTriggerVal, 2), leftTriggerVal);
    rightTriggerVal = Math.copySign(Math.pow(rightTriggerVal, 2), rightTriggerVal);
    stickVal = Math.copySign(Math.pow(stickVal, 2), stickVal);

    /**
     * Left and right triggers will cancel each other out.
     * Multiply X and Z inputs by the maximum speed attainable by the drivetrain's motors (raw units/100ms).
    */
    double xVelocity = (leftTriggerVal + rightTriggerVal) * DRIVETRAIN_PEAK_SPEED;
    double zVelocity = stickVal * DRIVETRAIN_PEAK_SPEED;

    /**
     * Return the maximum input to be either from the X or Z input velocities (whichever is larger),
     * but keep the sign (direction) of the X motion
     */
    double maxInput = Math.copySign(Math.max(Math.abs(xVelocity), Math.abs(zVelocity)), xVelocity);

    /**
     * Assign the final velocity to pass to the left and right side of the differential drivetrain.
     * Goes 'quadrant by quadrant' <-- xVelocity is y-axis, zVelocity is x-axis
     */
    if (xVelocity >= 0.0) {
      // First quadrant, else second quadrant
      if (zVelocity >= 0.0) {
        leftVelocity = maxInput;
        rightVelocity = xVelocity - zVelocity;
      } else {
        leftVelocity = xVelocity + zVelocity;
        rightVelocity = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zVelocity >= 0.0) {
        leftVelocity = xVelocity + zVelocity;
        rightVelocity = maxInput;
      } else {
        leftVelocity = maxInput;
        rightVelocity = xVelocity - zVelocity;
      }
    }

    /**
     * Use a closed loop velocity function to set the drivetrain's left- and right-side target
     * velocities to the derived velocities from above. Make sure that the derived velocities are
     * still within the barriers of the drivetrain motors' attainable velocities.
     */
    drivetrain_subsystem.setLeftVelocity(MathUtil.clamp(leftVelocity, -DRIVETRAIN_PEAK_SPEED, DRIVETRAIN_PEAK_SPEED));
    drivetrain_subsystem.setRightVelocity(MathUtil.clamp(rightVelocity, -DRIVETRAIN_PEAK_SPEED, DRIVETRAIN_PEAK_SPEED));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // If this Command ends, set the closed loop target velocity to zero.
    drivetrain_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
