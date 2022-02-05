// This program is used to demonstrate the setting of PID parameters
//   using the Spark Max and a small NEO motor.
// Shuffleboard allows adjustment of kF, kP, kI, and kD
// Shuffleboard displays joystick value, set (desired) speed,
//   and actual speed. 
// Also displayed is the actual percent difference between
//    set speed and actual speed. A boolean box turns green
//    when the 2 values are within 2 percent of each other
//  
// The joystick can be used to adjust the speed of the motor
//    to try out new parameters for kF, kP, kI and kD.
//  The trigger on the joystick (button 1) instantly doubles
//    the current set speed. A timer then displays the time
//    it takes to see the motor at within 2% of the new
//    set speed. This feature allows for tuning of the 
//    various parameters. 

//  Most sources say to leave kD at 0 for velocity PIDs. 

//  Suggest:
//    1. Set all parameters to 0 and see that motor does not run.
//    2. Adjust kF and see how close you can get to good tracking
//    3. Next, adjust kP to achieve better tracking and response.
//    4. If you see that the error is consistently centered on 
//        a non-zero value, kI may be set to correct the error 
//        toward 0.  This is mainly for obsessive individuals. d
//
//  Note:  The "setClosedLoopRampRate" parameter is useful to
//    avoid scaring the crap out of you if your parameters are
//    too large - PID algorithms are notorious for very dramatic,
//    noisy and possibly destructive behavior if the parameters
//    are too large. Always start with VERY SMALL parameter values
//    and work up. 

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// URL for downloading/installing the REV Spark Max Libraries
//  https://www.revrobotics.com/content/sw/max/sdk/REVRobotics.json

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private Joystick m_stick;
  private boolean measuring = false;
  private long startMS = 0;
  private double joyYAdjusted; 
  private double setPoint;


  private static final int deviceID = 40;
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private double baseY;
  private int UltraPort = 0;
  private int PhotoPort = 0;
  MedianFilter m_filter = new MedianFilter(10);
  private AnalogInput distance = new AnalogInput(UltraPort);
  private DigitalInput photocell = new DigitalInput(PhotoPort);
  
 


  @Override
  public void robotInit() {
    m_stick = new Joystick(0);

    // initialize motor
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();

    // the ramp rate is the limit on how fast the controller
    //    can change the speed of the motor - leaving this off
    //    or setting it too low can stress the gearbox, belts,
    //    or mechanisms. ".3" is 0 to full speed in 300MS
    m_motor.setClosedLoopRampRate(.3);

    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motor.getPIDController();

    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    //kP = 6e-5;
    kP = .0001;
    kI = 0;
    kD = 0; 
    kIz = 0; 
   // kFF = 0.000015;
    kFF  = .0001; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("Feed Forward", kFF);

    baseY =  m_stick.getY();    // set neutral joystick

  }

  @Override
  public void teleopInit() {
    // TODO Auto-generated method stub
    super.teleopInit();
      // PID coefficients
    kP = .0001;
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF  = .0001; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("Feed Forward", kFF);

    baseY =  m_stick.getY();    // set neutral joystick


  }
  @Override
  public void teleopPeriodic() {
    double percentError = 0;
    double distancevalue = 0;
    boolean photovalue = false;
  
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
  
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i / 1000000.0); kI = i; }
    if((d != kD)) { m_pidController.setD(d / 1000000.0); kD = d; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    
    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.ControlType.kDutyCycle
     *  com.revrobotics.ControlType.kPosition
     *  com.revrobotics.ControlType.kVelocity
     *  com.revrobotics.ControlType.kVoltage
     */

    percentError = Math.abs((setPoint - m_encoder.getVelocity())/setPoint);
    joyYAdjusted = m_stick.getY() - baseY;
    if (m_stick.getRawButton(1))
     {
       if(measuring)
       {  
          if ((percentError < .02) && (startMS != 0))
          {
            SmartDashboard.putNumber("React MS", System.currentTimeMillis() - startMS);
            startMS = 0;
          }
                                                                               
       }
       else
       {
         measuring = true;
         startMS = System.currentTimeMillis();
         setPoint = joyYAdjusted * maxRPM;
         m_pidController.setReference(setPoint, ControlType.kVelocity);
         double newY = joyYAdjusted * 2.0;
         if (newY > .8) newY = .8;
         if (newY < -.8) newY = -.8;
         setPoint = newY * maxRPM;
         m_pidController.setReference(setPoint, ControlType.kVelocity);
       }
     }

     // If joystick button is not down, use the joystick position as
     //   the setPoint.
    else
     {
       measuring = false;
       startMS = 0;
       setPoint = joyYAdjusted * maxRPM;
       m_pidController.setReference(setPoint, ControlType.kVelocity);
     } 
    SmartDashboard.putNumber("Joy Y", m_stick.getY() - baseY);
    SmartDashboard.putNumber("Target Speed", setPoint);
    SmartDashboard.putNumber("Actual Speed", m_encoder.getVelocity());
    SmartDashboard.putNumber("Graph", m_encoder.getVelocity());
    double speederrorpercent = 0;
    if (setPoint != 0) speederrorpercent = (setPoint - m_encoder.getVelocity())/setPoint * 100.0;
    SmartDashboard.putNumber("Speed Error", speederrorpercent);
    if (Math.abs((setPoint - m_encoder.getVelocity())/setPoint) <.02)
      SmartDashboard.putBoolean("Tracking", true);
    else SmartDashboard.putBoolean("Tracking", false);

    distancevalue = m_filter.calculate(distance.getVoltage());
  
    photovalue = photocell.get();
    SmartDashboard.putNumber("distance", distancevalue);
    SmartDashboard.putBoolean("Photo", photovalue);
    
  }
}
