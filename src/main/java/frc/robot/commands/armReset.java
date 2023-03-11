// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import frc.robot.subsystems.ArmAndJoint;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.revrobotics.CANSparkMax;
/** An example command that uses an example subsystem. */
public class armReset extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmAndJoint m_subsystem;
  private boolean check = false;
  private boolean check2 = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public armReset(ArmAndJoint subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    m_subsystem.m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!check) {
      m_subsystem.m_armMotor.set(-0.1);
      if(m_subsystem.armlimitSwitch.get()){
        m_subsystem.m_armMotor.set(0);
        m_subsystem.m_armEncoder.setPosition(0);
        check = true;
      }
    }

    // if reach limit switch
    // check boolean runs when decreasing encoder (negative value)
    if(check){
      m_subsystem.m_armMotor.set(0.1);

      // check if reach certain position and set arm speed to 0 
      // once it reaches it
      if(m_subsystem.m_armEncoder.getPosition() > 3) {
        check2 = true;
        m_subsystem.m_armMotor.set(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.m_armMotor.set(0);
    m_subsystem.m_armEncoder.setPosition(0);
    m_subsystem.m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    m_subsystem.m_armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return check2;
  }
}
