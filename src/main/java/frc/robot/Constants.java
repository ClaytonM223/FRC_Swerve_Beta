// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class Swerve{
        public static final int FRS = 0;    // Front Right Steer
        public static final int FRD = 0;    // Front Right Drive
        public static final int FLS = 0;    // Front Left Steer
        public static final int FLD = 0;    // Front Left Drive
        public static final int BRS = 0;    // Front Right Steer
        public static final int BRD = 0;    // Front Right Drive
        public static final int BLS = 0;    // Front Left Steer
        public static final int BLD = 0;    // Front Left Drive
    }

    public final class USB{
        public static final int DRIVER_CONTROLLER = 0;      // Driver Controller USB ID
        public static final int OPERATOR_CONTROLLER = 0;    // Operator controller USB ID
    }

    public final class Sensors{
        public static final int FR_ANGLE = 0; 
        public static final int FL_ANGLE = 0;
        public static final int BR_ANGLE = 0;
        public static final int BL_ANGLE = 0;
    }

    public final class ModuleConstants{
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 5.8462;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }
}
