/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class Limelight {
    private NetworkTable table_limelight;
    private NetworkTableEntry tv, tx, ty, ledMode, camMode, pipeline;

    private ShuffleboardTab tab_vision = Shuffleboard.getTab("Vision");
    private NetworkTableEntry distanceEntry = tab_vision.add("Distance to Target", 0).getEntry();
    private NetworkTableEntry tyEntry = tab_vision.add("ty value", 0).getEntry();

    public Limelight(String table_name) {
        //table_limelight = NetworkTableInstance.getDefault().getTable("limelight");
        table_limelight = NetworkTableInstance.getDefault().getTable(table_name);
        tv = table_limelight.getEntry("tv");
        tx = table_limelight.getEntry("tx");
        ty = table_limelight.getEntry("ty");
        ledMode = table_limelight.getEntry("ledMode");
        camMode = table_limelight.getEntry("camMode");
        pipeline = table_limelight.getEntry("pipeline");
    }

    public NetworkTable getNetworkTable() {
        return table_limelight;
    }

    /**
     * Returns if the Limelight has found any valid targets
     */
    public boolean foundTarget() {
        // Get the data from the Network Table (returns either 1 or 0)
        int data = tv.getNumber(0.0).intValue();
        // convert the data into a boolean (1 == true, 0 == false)
        boolean target_found = data == 1? true : false;

        return target_found;
    }

    public double get_tx() {
        return tx.getDouble(0);
    }

    /**
     * The input parameter of height is the height of the target
     * above ground level.
     */
    public double getDistanceToTarget(double height) {
        // the height of the target
        double h2 = height;
        // height of camera above floor
        double h1 = VISION_HEIGHT_OF_LIMELIGHT;
        // mounting angle of camera
        double a1 = VISION_ANGLE_OF_LIMELIGHT;
        // angle to the target from Limelight center (in degrees)
        double a2 = ty.getDouble(0.0);

        // KNOWN EQUATION: tan(a1+a2) = (h2-h1) / d
        // where d is the distance along a horizontal plane
        // Math.tan(double a) takes a parameter of radian values
        double d = (h2-h1) / Math.tan(Math.toRadians(a1+a2));

        return d;
    }

    public void putValuesToShuffleboard() {
        distanceEntry.setDouble(getDistanceToTarget(VISION_HEIGHT_OF_POWER_PORT_TARGET));
        tyEntry.setDouble(ty.getDouble(0.0));
    }

    public void setLedMode(int mode) {
        ledMode.setNumber(mode);
    }

    /**
     * Sets the mode of the Limelight.
     * If the parameter is set to true, then put the Limelight in
     * vision processing mode.
     * If the parameter is set to false, then put the Limelight in
     * driving mode (Increases exposure, disables vision processing).
     */
    public void setCameraMode(boolean mode) {
        // converts a boolean to an integer
        int number = mode? 1 : 0;
        camMode.setNumber(number);
    }

    // Returns the Limelight's current pipeline
    public int getCurrentPipeline() {
        int current_pipeline = pipeline.getNumber(0.0).intValue();
        return current_pipeline;
    }

    // Sets the Limelight to a new pipeline
    public void setCurrentPipeline(int new_pipeline) {
        pipeline.setNumber(new_pipeline);
    }

}
