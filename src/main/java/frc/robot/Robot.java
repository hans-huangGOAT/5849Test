/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_Stick;
  VictorSPX LFmotor;
  VictorSPX LMmotor;
  VictorSPX LRmotor;
  VictorSPX RFmotor;
  VictorSPX RMmotor;
  VictorSPX RRmotor;


  @Override
  public void robotInit() {
    LFmotor =new VictorSPX(0);
    LMmotor =new VictorSPX(1);
    LRmotor =new VictorSPX(2);
    RFmotor = new VictorSPX(3);
    RMmotor = new VictorSPX(4);
    RRmotor = new VictorSPX(5);

    m_Stick = new Joystick(0);
    LFmotor.setInverted(false);
    
  }

  @Override
  public void teleopPeriodic() {
   // m_myRobot.tankDrive(-0.7*m_Stick.getRawAxis(1), 0.7*m_Stick.getRawAxis(3));
   LFmotor.set(ControlMode.PercentOutput,m_Stick.getRawAxis(3) );
   LMmotor.set(ControlMode.PercentOutput,m_Stick.getRawAxis(3) );
   LRmotor.set(ControlMode.PercentOutput,m_Stick.getRawAxis(3) );
   RFmotor.set(ControlMode.PercentOutput,-m_Stick.getRawAxis(1) );
   RMmotor.set(ControlMode.PercentOutput,-m_Stick.getRawAxis(1) );
   RRmotor.set(ControlMode.PercentOutput,-m_Stick.getRawAxis(1) );

  }
}
