// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivebase extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final CANSparkMax m_leftMaster = new CANSparkMax(Constants.MotorControllerIDs.LEFT_MASTER, MotorType.kBrushless);
  private final CANSparkMax m_leftSlave = new CANSparkMax(Constants.MotorControllerIDs.LEFT_SLAVE, MotorType.kBrushless);

  private final CANSparkMax m_rightMaster = new CANSparkMax(Constants.MotorControllerIDs.RIGHT_MASTER, MotorType.kBrushless);
  private final CANSparkMax m_rightSlave = new CANSparkMax(Constants.MotorControllerIDs.RIGHT_SLAVE, MotorType.kBrushless);

  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftMaster, m_leftSlave);
  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightMaster, m_rightSlave);

  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftGroup, m_rightMaster);

  AHRS m_gyro = new AHRS(Port.kMXP);

  public Drivebase() {
    m_rightGroup.setInverted(true);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
