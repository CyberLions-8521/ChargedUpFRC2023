// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivebase extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //private Field2d m_field = new Field2d();

  //public DifferentialDriveKinematics m_kinematics = Constants.DriveConstants.kDriveKinematics;

  //private final DifferentialDriveOdometry m_odometry;

  private final CANSparkMax m_leftMaster = new CANSparkMax(Constants.MotorControllerIDs.LEFT_MASTER, MotorType.kBrushless);
  private final CANSparkMax m_leftSlave = new CANSparkMax(Constants.MotorControllerIDs.LEFT_SLAVE, MotorType.kBrushless);

  private final CANSparkMax m_rightMaster = new CANSparkMax(Constants.MotorControllerIDs.RIGHT_MASTER, MotorType.kBrushless);
  private final CANSparkMax m_rightSlave = new CANSparkMax(Constants.MotorControllerIDs.RIGHT_SLAVE, MotorType.kBrushless);

  private final RelativeEncoder m_rightEncoder = m_rightSlave.getEncoder();
  private final RelativeEncoder m_leftEncoder = m_leftMaster.getEncoder();

  public final MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftMaster, m_leftSlave);
  public final MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightMaster, m_rightSlave);

  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  //private final AHRS m_gyro = new AHRS(Port.kMXP);

  public Drivebase() {
    m_rightGroup.setInverted(true);
    //m_gyro.reset();
    //resetEncoders();

    // parameters
    // angle, distance measured by left encoder, distance measured by right encoder
    /*m_odometry =
    new DifferentialDriveOdometry(
        m_gyro.getRotation2d(), getLeftDistanceInch(), getRightDistanceInch());*/
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void arcadeDrive(double xSpeed, double zSpeed){
    m_diffDrive.arcadeDrive(xSpeed, zSpeed);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
/* 
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveKinematics getKinematics(){
    return m_kinematics;
  }
 */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }
/* 
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), getLeftDistanceInch(), getRightDistanceInch(), pose);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //getVelocity ?? = getRate (from encoder class)
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  public double getLeftDistanceInch() {
    return (m_leftEncoder.getPosition() * Math.PI * Constants.DriveConstants.kWheelDiameter);
  }

  public double getRightDistanceInch() {
    return (m_rightEncoder.getPosition() * Math.PI * Constants.DriveConstants.kWheelDiameter);
  }

  public double getAverageDistanceInch(){
    return ((getLeftDistanceInch() + getRightDistanceInch()) / 2);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftGroup.setVoltage(leftVolts);
    m_rightGroup.setVoltage(rightVolts);
    m_diffDrive.feed();
  }*/

  public void tankDrive(double leftSpeed, double rightSpeed){
    m_leftGroup.set(leftSpeed);
    m_rightGroup.set(rightSpeed);
  }
  /* 

  public void setMaxOutput(double maxOutput) {
    m_diffDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -m_gyro.getRate();
  }
  */
  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    /*m_odometry.update(
      m_gyro.getRotation2d(), getLeftDistanceInch(), getRightDistanceInch());
    m_field.setRobotPose(m_odometry.getPoseMeters());*/
    SmartDashboard.putNumber("rotations of left", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("rate of left", Math.abs(m_leftEncoder.getVelocity()));
    SmartDashboard.putNumber("rotations of right", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("rate of right", m_rightEncoder.getVelocity());
  } 

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
