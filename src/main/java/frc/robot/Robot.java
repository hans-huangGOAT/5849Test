/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

import java.text.DecimalFormat;

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
  Encoder encoderL;
  Encoder encoderR;
  ADXRS450_Gyro gyro;
  DecimalFormat formater = new DecimalFormat("00.00");
String gyro_Angle;
String encoder_rate_l;
String encoder_rate_r;
Accelerometer ac;
String x_ac;
String y_ac;
String z_ac;
double prevXAccel = 0;
double prevYAccel = 0;
SpeedControllerGroup a;

  @Override
  public void robotInit() {
    LFmotor =new VictorSPX(0);
    LMmotor =new VictorSPX(1);
    LRmotor =new VictorSPX(2);
    RFmotor = new VictorSPX(3);
    RMmotor = new VictorSPX(4);
    RRmotor = new VictorSPX(5);

    encoderL = new Encoder(2, 3, false, Encoder.EncodingType.k2X);
    encoderR = new Encoder(0, 1, true, Encoder.EncodingType.k2X);
    encoderR.setDistancePerPulse(4./256.);
    encoderL.setDistancePerPulse(4./256.);
    gyro = new ADXRS450_Gyro();
    ac = new BuiltInAccelerometer();

    m_Stick = new Joystick(0);
    LFmotor.setInverted(false);

  }
  @Override
  public void teleopInit() {
    // TODO Auto-generated method stub
    super.teleopInit();
    encoderR.reset();
    encoderL.reset();
    gyro.reset();
  }

  @Override
  public void teleopPeriodic() {
    double yAccel = ac.getY();
    double yJerk = (yAccel - prevYAccel)/.02;
    prevYAccel = yAccel;

   // m_myRobot.tankDrive(-0.7*m_Stick.getRawAxis(1), 0.7*m_Stick.getRawAxis(3));
   LFmotor.set(ControlMode.PercentOutput,m_Stick.getRawAxis(3) );
   LMmotor.set(ControlMode.PercentOutput,m_Stick.getRawAxis(3) );
   LRmotor.set(ControlMode.PercentOutput,m_Stick.getRawAxis(3) );
   RFmotor.set(ControlMode.PercentOutput,-m_Stick.getRawAxis(1) );
   RMmotor.set(ControlMode.PercentOutput,-m_Stick.getRawAxis(1) );
   RRmotor.set(ControlMode.PercentOutput,-m_Stick.getRawAxis(1) );
   gyro_Angle = formater.format(gyro.getAngle());
   encoder_rate_l = formater.format(encoderR.getRate());
   encoder_rate_r = formater.format(encoderL.getRate());
   x_ac = formater.format(ac.getX());
   y_ac = formater.format(yJerk);
   z_ac = formater.format(ac.getZ());
   System.out.println("Left Rate: " + encoder_rate_l+ " Right Rate: "+ encoder_rate_r+" Gyro Angle: "+ gyro_Angle+"            "+y_ac);
  }
}
