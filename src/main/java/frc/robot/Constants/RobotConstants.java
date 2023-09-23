// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.util.Units;
import frc.robot.Classes.ModuleConfig;
import frc.robot.Constants.Constants.ROBOT_TYPE;

//Switch between the WASP and NEO by commenting out the other one

/** This is the constants for the WASP */
// public class RobotConstants {
//     public static final ROBOT_TYPE ROBOT = ROBOT_TYPE.WASP;

//     public static final double WHEEL_WIDTH = Units.inchesToMeters(18.75); //Make sure this is from the wheel's center of rotation
//     public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4); //TODO
    
//     public static final ModuleConfig[] MOD_CONFIGS = {
//         new ModuleConfig(1, 2, 9, 0.0),
//         new ModuleConfig(3, 4, 10, 0.0),
//         new ModuleConfig(5, 6, 11, 0.0),
//         new ModuleConfig(7, 8, 12, 0.0)
//     };

//     public static final int PIGEON_ID = 0;

//     public static final double MAX_SPEED = 4.0; //TODO
//     public static final double MAX_ANGULAR_VELOCITY = 0.0; //TODO

//     // SHOOTER

//     public static final int FLYWHEEL_L_ID = 0; //TODO
//     public static final int FLYWHEEL_R_ID = 0; //TODO
//     public static final int BACK_FLYWHEEL_ID = 0; //TODO
//     public static final int FEED_ID = 0; //TODO
//     public static final int BELT_ID = 0; //TODO

//     /** Hood's angle of elevation in degrees */
//     public static final double HOOD_ANGLE = 45.0; //TODO
// }

/** This is the constants for the NEO */
public class RobotConstants {
    public static final ROBOT_TYPE ROBOT = ROBOT_TYPE.NEO;

    public static final double WHEEL_WIDTH = Units.inchesToMeters(18.75); //Make sure this is from the wheel's center of rotation
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4); //TODO
    
    public static final ModuleConfig[] MOD_CONFIGS = {
        new ModuleConfig(1, 2, 9, 0.0),
        new ModuleConfig(3, 4, 10, 0.0),
        new ModuleConfig(5, 6, 11, 0.0),
        new ModuleConfig(7, 8, 12, 0.0)
    };

    public static final int PIGEON_ID = 0;

    public static final double MAX_SPEED = 4.0; //TODO
    public static final double MAX_ANGULAR_VELOCITY = 0.0; //TODO

    // SHOOTER

    public static final int FLYWHEEL_M_ID = 0; //TODO
    public static final int FLYWHEEL_F_ID = 0; //TODO
    public static final int BACK_FLYWHEEL_ID = 0; //TODO
    public static final int FEED_ID = 0; //TODO
    public static final int BELT_ID = 0; //TODO

    //PID Consants
    public static final double FLYWHEEL_P = 1; //TODO
    public static final double FLYWHEEL_I = 0; //TODO
    public static final double FLYWHEEL_D = 0; //TODO
    //Feedforward Constants
    public static final double FLYWHEEL_KS = 1; //TODO
    public static final double FLYWHEEL_KV = 0; //TODO
    public static final double FLYWHEEL_KA = 0; //TODO

    /** Hood's angle of elevation in degrees */
    public static final double HOOD_ANGLE = 45.0; //TODO
}