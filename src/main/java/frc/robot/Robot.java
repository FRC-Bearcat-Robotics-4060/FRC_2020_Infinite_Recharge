/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
//import frc.robot.imports.*;
public class Robot extends TimedRobot {
  
  AHRS navx;
  Joystick _joystick1 = new Joystick(0);
  CANSparkMax _leftBackCanSparkMax = new CANSparkMax((1), MotorType.kBrushless);

  CANSparkMax _leftFrontCanSparkMax = new CANSparkMax((4), MotorType.kBrushless);

  CANSparkMax _rightBackCanSparkMax = new CANSparkMax((2), MotorType.kBrushless);


  CANSparkMax _rightFrontCanSparkMax = new CANSparkMax((3), MotorType.kBrushless); 


  CANSparkMax _collectVert = new CANSparkMax((13), MotorType.kBrushless);

  CANSparkMax _shooterMotorLeft = new CANSparkMax((14), MotorType.kBrushless);

  CANSparkMax _shooterMotorRight = new CANSparkMax((15), MotorType.kBrushless); 

  //CANSparkMax _miscSpark = new CANSparkMax((null), MotorType.kBrushless);
  
  private SpeedControllerGroup m_LeftMotors = new SpeedControllerGroup(_leftBackCanSparkMax, _leftFrontCanSparkMax);
  private SpeedControllerGroup m_RightMotors = new SpeedControllerGroup(_rightBackCanSparkMax, _rightFrontCanSparkMax);
 
  private DifferentialDrive m_Drive = new DifferentialDrive(m_LeftMotors, m_RightMotors);

  TalonSRX _colorWheelTalon = new TalonSRX(10);

  TalonSRX _ColectorMotor = new TalonSRX(11);

  TalonSRX _liftmotor = new TalonSRX(12);

  Spark _magMotor1 = new Spark(0);


  Spark _magMotor2 = new Spark(1);
  



  Boolean directionFlipped = false;
  Boolean button12Toggle = false;
  Boolean button11Toggle = false;
  Boolean speedToggle = false;
  Boolean lihtAct = false;
  double _tavar = 0.0;
  Boolean lightspeed = false;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

   private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

   private final ColorMatch m_colorMatcher = new ColorMatch();

   private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
   private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
   private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
   private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  NetworkTableEntry ledEntry;
  NetworkTableEntry camMode;

  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;




  
@Override
  public void robotInit() {

    CameraServer.getInstance().startAutomaticCapture();

    _leftBackCanSparkMax.setClosedLoopRampRate(0);
    _leftFrontCanSparkMax.setClosedLoopRampRate(0);
    _rightBackCanSparkMax.setClosedLoopRampRate(0);
    _rightFrontCanSparkMax.setClosedLoopRampRate(0);

   
    _leftBackCanSparkMax.setIdleMode(IdleMode.kCoast);
    _leftFrontCanSparkMax.setIdleMode(IdleMode.kCoast);
    _rightBackCanSparkMax.setIdleMode(IdleMode.kCoast);
    _rightFrontCanSparkMax.setIdleMode(IdleMode.kCoast);

   m_colorMatcher.addColorMatch(kBlueTarget);
   m_colorMatcher.addColorMatch(kGreenTarget);
   m_colorMatcher.addColorMatch(kRedTarget);
   m_colorMatcher.addColorMatch(kYellowTarget);  

   try {
    /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
    /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
    /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
    navx = new AHRS(SPI.Port.kMXP); 
    navx.enableLogging(true);
} catch (RuntimeException ex ) {
    DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);

  }
}

  @Override
  public void robotPeriodic() {

    navxReadout();

   CANEncoder leftBack_encoder = _leftBackCanSparkMax.getEncoder();
   CANEncoder rightBack_encoder = _rightBackCanSparkMax.getEncoder();
   

   
   SmartDashboard.putBoolean("SpeedToggle", speedToggle);
    SmartDashboard.putBoolean("LightSpeed", lightspeed);
   SmartDashboard.putNumber("Left Encoder", leftBack_encoder.getPosition());
    SmartDashboard.putNumber("Right Encoder", rightBack_encoder.getPosition());


   SmartDashboard.putNumber("Left Encoder_Graph", leftBack_encoder.getPosition());
   SmartDashboard.putNumber("Right Encoder_Graph", rightBack_encoder.getPosition());

    Color detectedColor = m_colorSensor.getColor();
    String colorString;
    String fieldColor;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
      fieldColor = "Red";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
      fieldColor = "Blue";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
      fieldColor = "Yellow";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
      fieldColor = "Green";
    } else {
      colorString = "Unknown";
      fieldColor = "IDunno";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putString("Field Color", fieldColor);

    SmartDashboard.putNumber("Joystick Forward", -_joystick1.getY());
    SmartDashboard.putNumber("Robot Forward", _leftFrontCanSparkMax.getAppliedOutput());
  
    
  
  
  
  }







  @Override 
  public void teleopInit() {
    directionFlipped = true;
    speedToggle = false;
    Update_Limelight_Tracking();

    ledEntry.setDouble(1);

    camMode.setDouble(1);

 
  }




@Override
public void teleopPeriodic() {
  buttonToggles();

  double collectorDirection = -_joystick1.getRawAxis(3);

  double deadzone = 0.3;

  double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  
  double _joyforwardRaw = -_joystick1.getZ();
  double _joyrotateRaw = -_joystick1.getY() ;


  double _joyforward ;
  double _joyrotate ;

  if (_joyforwardRaw > deadzone || _joyforwardRaw < -deadzone) {
     _joyforward = _joyforwardRaw;
   }
  

   if (_joyrotateRaw > deadzone || _joyrotateRaw < -deadzone) {
    _joyrotate = _joyrotateRaw;
  }

  double forward2 = _joyforwardRaw * _joyforwardRaw * (_joyforwardRaw < 0 ? -1.0 : 1.0);
      double rotate2 = _joyrotateRaw * _joyrotateRaw * (_joyrotateRaw < 0 ? -0.8 : 0.8);

      double forwardPower = 0.33 * Math.abs(forward2) >= 0.1 ? forward2 : 0;
      double rotatePower = 0.33 * Math.abs(rotate2) >= 0.1 ? rotate2 : 0;

      double drive_left = (rotatePower - forwardPower) / 3;
      double drive_right = (rotatePower + forwardPower) / 3;

      SmartDashboard.putNumber("Left Drive", drive_left);

      SmartDashboard.putNumber("Right Drive", drive_right);

    

 if (_joystick1.getRawButton(3)) {
limelightAutonomous();
 }

else if (speedToggle == true && lightspeed == false) {
  m_Drive.tankDrive(drive_left * 1.7, drive_right * 1.7);
}

else if (lightspeed == true) {
  //m_Drive.tankDrive(drive_left * 3, drive_right * 3);

}

 else {
  ledEntry.setDouble(1);

  camMode.setDouble(1);
m_Drive.tankDrive(drive_left * 1.3, drive_right * 1.3);
 }
      
if (_joystick1.getRawButton(4)) {
_liftmotor.set(ControlMode.PercentOutput, 0.4);

_liftmotor.set(ControlMode.PercentOutput, -0.4);
}
else if (_joystick1.getRawButton(6)) {

  _liftmotor.set(ControlMode.PercentOutput, -0.4);

  _liftmotor.set(ControlMode.PercentOutput, 0.4);
}
else {

  _liftmotor.set(ControlMode.PercentOutput, 0);

  _liftmotor.set(ControlMode.PercentOutput, 0);
}


//For gathering
//Read current from powerboard for shooter 
//Trigger + Thumb = Shooting
//Thumb = Magazine loading 
//Trigger = Collector 
//Direction 
//Down
if (collectorDirection > 0.1) {


if (_joystick1.getRawButton(1)) {


_ColectorMotor.set(ControlMode.PercentOutput, -0.75);




_collectVert.set(-0.95);

// _magMotor1.set(1);
// _magMotor2.set(-1);

// _shooterMotorLeft.set(0.90);
// _shooterMotorRight.set(-0.90);


}
else if (_joystick1.getRawButton(2)) {

 _magMotor1.set(1);
 _magMotor2.set(-1);
}
else if (_joystick1.getRawButton(1) && _joystick1.getRawButton(2)) {
  _shooterMotorLeft.set(0.90);
_shooterMotorRight.set(-0.90);

}
else {
  
  _ColectorMotor.set(ControlMode.PercentOutput, 0);

  _magMotor1.set(0);
  _magMotor2.set(0);

  _collectVert.set(0);
  _shooterMotorLeft.set(0);
  _shooterMotorRight.set(0);
  
}

}

//Up
else if (collectorDirection < -0.1) {


  if (_joystick1.getRawButton(1)) {
  
  
  _ColectorMotor.set(ControlMode.PercentOutput, 0.75);
  
  
  
  
  _collectVert.set(0.95);
  
  
  
  }
  else if (_joystick1.getRawButton(2)) {
  
   _magMotor1.set(-1);
   _magMotor2.set(1);
  }
  else if (_joystick1.getRawButton(1) && _joystick1.getRawButton(2)) {
    _shooterMotorLeft.set(-0.75);
  _shooterMotorRight.set(0.75);
  
  }
  else {
    
    _ColectorMotor.set(ControlMode.PercentOutput, 0);
  
    _magMotor1.set(0);
    _magMotor2.set(0);
  
    _collectVert.set(0);
    _shooterMotorLeft.set(0);
    _shooterMotorRight.set(0);
    
  }
  }






//Color Wheel
if (_joystick1.getRawButton(5)) {
  _colorWheelTalon.set(ControlMode.PercentOutput, 1);
}
 
else if (_joystick1.getRawButton(3)) {
  _colorWheelTalon.set(ControlMode.PercentOutput, -1);
}   

else {

  _colorWheelTalon.set(ControlMode.PercentOutput, 0);
}



}     
  






public void Update_Limelight_Tracking() {
  // These numbers must be tuned for your Robot! Be careful!
  final double STEER_K = 0.02; // how hard to turn toward the target
  final double DRIVE_K = 0.16; // how hard to drive fwd toward the target
  final double DESIRED_TARGET_AREA = 13.0; // Area of the target when the robot reaches the wall
  final double MAX_DRIVE = 0.7; // Simple speed limit so we don't drive too fast

  double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  camMode = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode");
  ledEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode");
  // ty = ty - 8.00;
  ta = ta - 13.00;
  tx = tx + 4.1;
  //System.out.println(tv);
  if (tv != 1.0) {
    m_LimelightHasValidTarget = false;
    m_LimelightDriveCommand = 0.0;
    m_LimelightSteerCommand = 0.0;
    return;
  } else if (tv == 1.0) {

    m_LimelightHasValidTarget = true;

  }

  // Start with proportional steering
  double steer_cmd = tx * STEER_K;
  m_LimelightSteerCommand = steer_cmd;

  // try to drive forward until the target area reaches our desired area
  // double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
  double drive_cmd = ta * DRIVE_K;

  // don't let the robot drive too fast into the goal
  // if (drive_cmd > MAX_DRIVE)
  // {
  // drive_cmd = MAX_DRIVE;
  // }
  m_LimelightDriveCommand = drive_cmd / 5;

  _tavar = ta;
}

public void limelightAutonomous() {
  Update_Limelight_Tracking();
  ledEntry.setDouble(3);
  camMode.setDouble(0);
    double steer = _joystick1.getZ();
    double drive = _joystick1.getY();
    boolean auto = false;//_joystick1.getRawButton(8);

    steer *= 1;
    drive *= -1;

    if (!auto) {
      if (m_LimelightHasValidTarget) {
        m_Drive.arcadeDrive(-m_LimelightDriveCommand, m_LimelightSteerCommand);
       System.out.println("Drive" + -m_LimelightDriveCommand);

        System.out.println("Steer" + -m_LimelightSteerCommand);
      }

      else if (_tavar > 3.0) {
        
        m_Drive.arcadeDrive(-0.3, 0);

      } else {
        m_Drive.arcadeDrive(0.0, 0.0);
      }

    } else {
      m_Drive.arcadeDrive(drive / 2, steer / 2);
    }

}

public void limelightShooter() {
  Update_Limelight_Tracking();
  ledEntry.setDouble(3);
  camMode.setDouble(0);
    
    boolean auto = false;//_joystick1.getRawButton(8);

    

    if (!auto) {
      if (m_LimelightHasValidTarget) {
       //_miscSpark.set()
       System.out.println("Drive" + -m_LimelightDriveCommand);

        System.out.println("Steer" + -m_LimelightSteerCommand);
      }

      else if (_tavar > 3.0) {
        
       

      } else {
        
      }

    } else {
      
    }

}

public void buttonToggles() {
int speedbutton = 2;

  if (_joystick1.getRawButtonPressed(speedbutton)) {

if (!speedToggle) {
  speedToggle = true;
} 
 else if (speedToggle)  {
    speedToggle = false;
      }
    }

if (_joystick1.getTrigger() ) {
 //lightspeed = true;
  // if (!lightspeed) {
  // lightspeed = true;
  // } 
  //   else if (lightspeed) {
  //     lightspeed = false;
  //       }

  //     }
}
else {
  lightspeed = false;
}



}

public void navxReadout() {
  SmartDashboard.putBoolean(  "IMU_Connected",        navx.isConnected());
          SmartDashboard.putBoolean(  "IMU_IsCalibrating",    navx.isCalibrating());
          SmartDashboard.putNumber(   "IMU_Yaw",              navx.getYaw());
          SmartDashboard.putNumber(   "IMU_Pitch",            navx.getPitch());
          SmartDashboard.putNumber(   "IMU_Roll",             navx.getRoll());
          
          /* Display tilt-corrected, Magnetometer-based heading (requires             */
          /* magnetometer calibration to be useful)                                   */
          
          SmartDashboard.putNumber(   "IMU_CompassHeading",   navx.getCompassHeading());
          
          /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
          SmartDashboard.putNumber(   "IMU_FusedHeading",     navx.getFusedHeading());

          /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
          /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
          
          SmartDashboard.putNumber(   "IMU_TotalYaw",         navx.getAngle());
          SmartDashboard.putNumber(   "IMU_YawRateDPS",       navx.getRate());

          /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
          
          SmartDashboard.putNumber(   "IMU_Accel_X",          navx.getWorldLinearAccelX());
          SmartDashboard.putNumber(   "IMU_Accel_Y",          navx.getWorldLinearAccelY());
          SmartDashboard.putBoolean(  "IMU_IsMoving",         navx.isMoving());
          SmartDashboard.putBoolean(  "IMU_IsRotating",       navx.isRotating());

          /* Display estimates of velocity/displacement.  Note that these values are  */
          /* not expected to be accurate enough for estimating robot position on a    */
          /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
          /* of these errors due to single (velocity) integration and especially      */
          /* double (displacement) integration.                                       */
          
          SmartDashboard.putNumber(   "Velocity_X",           navx.getVelocityX());
          SmartDashboard.putNumber(   "Velocity_Y",           navx.getVelocityY());
          SmartDashboard.putNumber(   "Displacement_X",       navx.getDisplacementX());
          SmartDashboard.putNumber(   "Displacement_Y",       navx.getDisplacementY());
          
          /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
          /* NOTE:  These values are not normally necessary, but are made available   */
          /* for advanced users.  Before using this data, please consider whether     */
          /* the processed data (see above) will suit your needs.                     */
          
          SmartDashboard.putNumber(   "RawGyro_X",            navx.getRawGyroX());
          SmartDashboard.putNumber(   "RawGyro_Y",            navx.getRawGyroY());
          SmartDashboard.putNumber(   "RawGyro_Z",            navx.getRawGyroZ());
          SmartDashboard.putNumber(   "RawAccel_X",           navx.getRawAccelX());
          SmartDashboard.putNumber(   "RawAccel_Y",           navx.getRawAccelY());
          SmartDashboard.putNumber(   "RawAccel_Z",           navx.getRawAccelZ());
          SmartDashboard.putNumber(   "RawMag_X",             navx.getRawMagX());
          SmartDashboard.putNumber(   "RawMag_Y",             navx.getRawMagY());
          SmartDashboard.putNumber(   "RawMag_Z",             navx.getRawMagZ());
          SmartDashboard.putNumber(   "IMU_Temp_C",           navx.getTempC());
          
          /* Omnimount Yaw Axis Information                                           */
          /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
          //navx.BoardYawAxis yaw_axis = navx.getBoardYawAxis();
        //  SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
          //SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
          
          /* Sensor Board Information                                                 */
          SmartDashboard.putString(   "FirmwareVersion",      navx.getFirmwareVersion());
          
          /* Quaternion Data                                                          */
          /* Quaternions are fascinating, and are the most compact representation of  */
          /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
          /* from the Quaternions.  If interested in motion processing, knowledge of  */
          /* Quaternions is highly recommended.                                       */
          SmartDashboard.putNumber(   "QuaternionW",          navx.getQuaternionW());
          SmartDashboard.putNumber(   "QuaternionX",          navx.getQuaternionX());
          SmartDashboard.putNumber(   "QuaternionY",          navx.getQuaternionY());
          SmartDashboard.putNumber(   "QuaternionZ",          navx.getQuaternionZ());
          
          /* Connectivity Debugging Support                                           */
          SmartDashboard.putNumber(   "IMU_Byte_Count",       navx.getByteCount());
          SmartDashboard.putNumber(   "IMU_Update_Count",     navx.getUpdateCount());
}
}

