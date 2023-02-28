// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw extends SubsystemBase {
  /** Creates a new Arm. */
  public Claw() {
    m_doubleSolenoid.set(Value.kForward);
    comp.enableDigital();
  }

  //Type can eiher be REVPH or CTREPCM
  Compressor comp = new Compressor(PneumaticsModuleType.CTREPCM);
  DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

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
  public CommandBase Grab() {
    return runOnce(
        () -> {
          m_doubleSolenoid.set(Value.kForward);

          
        });
  }
  public CommandBase Release() {
    return runOnce(
        () -> {
          m_doubleSolenoid.set(Value.kReverse);
        });
  }
  public CommandBase setOff() {
    return runOnce(
        () -> {
          m_doubleSolenoid.set(Value.kOff);
        });
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}