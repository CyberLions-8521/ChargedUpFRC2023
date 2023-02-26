// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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
  public static final class ElevatorConstants{
    public static final double height = 5;
    public static final double r1 = 3;
  }

  public static final class DriveConstants{
    public static final double kTrackwidthMeters = Units.inchesToMeters(24);
    public static final double kWheelDiameter = Units.inchesToMeters(6);

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    
    public static final double ksVolts = 0.096808;
    public static final double kvVoltSecondsPerMeter = 2.811;
    public static final double kaVoltSecondsSquaredPerMeter = 0.11014;

    public static final double kPDriveVel = 0.0785;
    public static final double kDDriveVel = 0.0;
    public static final double kIDriveVel = 0.0;
  
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 2.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int amogus = 0;
  }

  public static class MotorControllerIDs{
    public static final int LEFT_MASTER = 3;
    public static final int LEFT_SLAVE = 4;

    public static final int RIGHT_MASTER = 2;
    public static final int RIGHT_SLAVE = 1;

    public static final int JOINT_MOTOR1 = 5;
    public static final int JOINT_MOTOR2 = 6;

    public static final int ARM_MOTOR = 30;
  }
}
