
package org.usfirst.frc.team5493.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;

/*
 * Mecanum drive using a single Logitech Extreme 3D Pro Joystick
 * and 4 drive motors connected through Victor 888 Controllers as follows:
 *     - PWM 2 - Connected to front left drive motor
 *     - PWM 3 - Connected to rear left drive motor
 *     - PWM 4 - Connected to front right drive motor
 *     - PWM 5 - Connected to rear right drive motor
 */

public class Robot extends SampleRobot {
	// Create a robot drive object using PWMs 2, 3, 4, 5
	Talon d_fleft = new Talon(2);// PWM port 2
	Victor d_rleft = new Victor(3);// PWM port 3
	Victor d_fright = new Victor(4);// PWM port 4
	Victor d_rright = new Victor(5);// PWM port 5
	RobotDrive mec_drive = new RobotDrive(d_fleft, d_rleft, d_fright, d_rright);
	// Define joystick being used at USB port 0 on the Driver Station
	Joystick mec_stick = new Joystick(0);
	// Define compressor & double solenoid
	Compressor compressor = new Compressor(0);
	DoubleSolenoid doublesolenoid = new DoubleSolenoid(7,4);
	// Define status indicator for Pneumatic Launcher
	private boolean isChambered;
// this is a test
	public void operatorControl() {
		while (true && isOperatorControl() && isEnabled()) {
			// DriverStation.getInstance().reportError("X="+mec_stick.getX(),
			// false);
			// DriverStation.getInstance().reportError("Y="+mec_stick.getY(),
			// false);
			// DriverStation.getInstance().reportError("T="+mec_stick.getTwist(),
			// false);
			mec_drive.mecanumDrive_Cartesian(mec_stick.getX(),mec_stick.getY(),mec_stick.getTwist(),0);
			// mec_drive.drive(0.2, 0);
			// Turn Compressor ON/OFF- PCM controls Compressor @ 120 psi automatically
			compressor.setClosedLoopControl(true);
			// Pneumatic Launcher
			if (mec_stick.getRawButton(1) && isChambered) {
				DriverStation.getInstance();
				DriverStation.reportError("LAUNCH",false);
				doublesolenoid.set(DoubleSolenoid.Value.kForward);
				isChambered = false;
				Timer.delay(0.5);
			} else if (mec_stick.getRawButton(1) && !isChambered) {
				DriverStation.getInstance();
				DriverStation.reportError("CHAMBER",false);
				doublesolenoid.set(DoubleSolenoid.Value.kReverse);
				isChambered = true;
				Timer.delay(0.5);
			} else {
				DriverStation.getInstance();
				DriverStation.reportError("OFF",false);
				doublesolenoid.set(DoubleSolenoid.Value.kOff);
			}
		}
	}
}