
package org.usfirst.frc.team5684.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PrintCommand;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;




/**
 * This is a demo program showing the use of the RobotDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */

public class Robot extends IterativeRobot {
	Thread visionThread;
	RobotDrive myRobot;
	Victor collector;
	Spark arm;
	CameraServer camera;
	Joystick stick;
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	SendableChooser chooser;
	boolean pully = false;
	boolean armDrive = false;
	double pullySpeed = 0.5;
	AnalogGyro myGyro;
	final double voltsPerDegreePerSecond = 0.0011;
	Encoder encoder;
	AnalogPotentiometer pot = new AnalogPotentiometer(0, 360, 30);
	public static final int CHIEBUKA=1;
	public static final int KOBY=2;
	private int driver;
	private int forward=1;
	private double turnSpeed=.5;
	public Robot() {
		driver =KOBY;
		myRobot = new RobotDrive(0, 1);
		myRobot.setExpiration(0.1);
		stick = new Joystick(0);
		collector = new Victor(2);
		arm = new Spark(3);
		myGyro = new AnalogGyro(1);
		encoder = new Encoder(0, 1, false, EncodingType.k1X); /// have to find
																/// right
																/// scaling
																/// factor
																/// (EncoderType)
	}

	public void robotInit() {
		
		visionThread = new Thread(() -> {
			// Get the UsbCamera from CameraServer
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// Set the resolution
			camera.setResolution(1280, 720);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 1280, 720);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat.  If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}
				// Put a rectangle on the image
				Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400),
						new Scalar(255, 255, 255), 5);
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
		visionThread.setDaemon(true);
		visionThread.start();
		
		
		
		
		chooser = new SendableChooser();
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto modes", chooser);
		//camera = CameraServer.getInstance();
		//camera.setQuality(85);
		myGyro.reset();
		myGyro.setSensitivity(voltsPerDegreePerSecond);
		myGyro.calibrate(); // calibrates gyro values to equal degrees

		// System.out.println("robotInit: gyro.getAngle = " +
		// myGyro.getAngle());

		// System.out.println("robotInit: setting encoder defaults");

		encoder.setMaxPeriod(0.1);
		encoder.setMinRate(10.0);
		encoder.setDistancePerPulse(5);
		encoder.setReverseDirection(true);
		encoder.setSamplesToAverage(7);
		System.out.println("robotInit: Finished setting encoder defaults");

		SmartDashboard.putString("DB/String 0", "NOT SET");

		System.out.println("robotInit");

		//camera.startAutomaticCapture("cam1");
		// myGyro.calibrate(); //calibrates gyro values to equal degrees
		// System.out.println("Gyro Angle: " + myGyro.getAngle());
		if(SmartDashboard.getBoolean("DB/Button 3")){
			driver=CHIEBUKA;
			SmartDashboard.putString("DB/String 5", "CHIEBUKA is driving");
		}
		else
		{
			driver=KOBY;
			SmartDashboard.putString("DB/String 5", "Koby is driving");
		}
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */

	public void autonomous() {
		/*
		 * Command autonomousCommand = (Command) chooser.getSelected();
		 * autonomousCommand.start();
		 */
		if (true == SmartDashboard.getBoolean("DB/Button 0") && false == SmartDashboard.getBoolean("DB/Button 1")) {
			System.out.println("Button 0 is true");
			;
			SmartDashboard.putString("DB/String 0", "Cheval de Frise");
			cheval_de_frise();
		} else if (false == SmartDashboard.getBoolean("DB/Button 0")
				&& true == SmartDashboard.getBoolean("DB/Button 1")) {
			System.out.println("Button 0 is false");
			;
			SmartDashboard.putString("DB/String 0", "Portcullis");
			portcullis();
		} else {
			System.out.println("Button 0 is false");
			;
			SmartDashboard.putString("DB/String 0", "Low Bar");
			low_bar();
		}

	}

	public void low_bar() {

		double rot_angle = 0.0;

		double requested_travel_distance = 0.0;
		myGyro.reset();
		myGyro.setSensitivity(voltsPerDegreePerSecond);
		myGyro.calibrate(); // Added by SRH at 15:11
		double driveForTime = 3500.0; // This is the time to drive AFTER the arm
										// is down
		double driveBeforeArm = 0.0; // This is the time to drive BEFORE the arm
										// is down

		double drivingScalingFactor = 1.2; // this is to affect the speed and
											// distance after the arm is down

		driveBeforeArm = 750.0; // Number of loops before arm goes down

		driveForTime = driveForTime - driveBeforeArm; // we need to drive
														// forward for a bit
														// before
		// the arm comes down. this way, we can tweak
		// the value and adjust the total distance traveled
		driveForTime = driveForTime / drivingScalingFactor; // this adjusts the
															// loop - speed is
															// adjusted below
		// higher speed means less loops

		System.out.println("driveForTime is " + driveForTime);
		System.out.println("driveBeforeArm is " + driveBeforeArm);

		/***************** Put the arm down **************************/

		Timer armTimer = new Timer();
		armTimer.start();

		while (!(armTimer.hasPeriodPassed(2.7))) // number passed in is in
													// seconds
		{
			arm.setInverted(true);

			arm.set(0.5);

			armDrive = true;

			Timer.delay(0.005);

			rot_angle = pot.get();

			System.out.println("In loop!");

			SmartDashboard.putNumber("Arm Angle", rot_angle);

			if (rot_angle >= 45.0) {
				System.out.println("Potentiometer has reached: " + pot.get() + " degrees. Breaking out of loop");

				// break;
			}
		}

		arm.set(0);

		/************************* Drive Forward ******************************/

		double Kp = 0.03;

		double turningValue;

		int print_counter = 0;

		double drivingLoops = 0.0;

		while (drivingLoops < driveBeforeArm) // ||
												// !(armTimer.hasPeriodPassed(15)))
		{
			int encoder_count = 0;

			turningValue = -myGyro.getAngle() * Kp; // THIS WILL USE THE GYRO
													// INPUT TO DETERMINE
													// TURNING ANGLE

			myRobot.drive(-0.22, turningValue); // THIS IS GOING FORWARDS
												// (COLLECTOR IN FRONT)

			drivingLoops++;

			Timer.delay(0.002);
			print_counter++;
			if ((print_counter % 200) == 0) {
				// System.out.println("Current GYRO Angle: " +
				// myGyro.getAngle());
			}

			if (print_counter % 50 == 0) {
				System.out.println("Distance: " + encoder.getDistance() + " Count: " + encoder.get() + " Raw: "
						+ encoder.getRaw());

				SmartDashboard.putNumber("Distance Traveled: ", encoder.getDistance());

				SmartDashboard.putNumber("Raw Count: ", encoder.get());

			}

			if (encoder.getDistance() >= requested_travel_distance) {
				// break; ///out of loop
			}
		}

		print_counter = 0;
		drivingLoops = 0.0;

		/// reset encoder to get it back to 0
		encoder.reset();

		Timer autonomousTimer = new Timer();

		autonomousTimer.start();

		double driveSpeed = 0.22 * drivingScalingFactor; // to change speed,
															// change scaling
															// factor above

		while (isAutonomous()) // || !(armTimer.hasPeriodPassed(15)))
		{
			turningValue = -myGyro.getAngle() * Kp; // THIS WILL USE THE GYRO
													// INPUT TO DETERMINE
													// TURNING ANGLE

			if (drivingLoops < driveForTime) {

				myRobot.drive(-driveSpeed, turningValue);

				drivingLoops++;
			}
			Timer.delay(0.002);
			print_counter++;
			if ((print_counter % 200) == 0) {
				// System.out.println("Current GYRO Angle: " +
				// myGyro.getAngle());
			}

			if (print_counter % 50 == 0) {
				// System.out.println("Distance: " + encoder.getDistance()+ "
				// Count: " + encoder.get() + " Raw: " + encoder.getRaw());

				SmartDashboard.putNumber("Distance Traveled: ", encoder.getDistance());

				SmartDashboard.putNumber("Raw Count: ", encoder.get());
			}

			if (encoder.getDistance() >= requested_travel_distance) {
				// break; ///out of loop
			}
		}
	}

	public void cheval_de_frise() {
		/// reset encoder
		double requested_travel_distance = 0.0;
		myGyro.reset();
		myGyro.setSensitivity(voltsPerDegreePerSecond);
		myGyro.calibrate(); // Added by SRH at 15:11
		double driveForTime = 0.0; // This is the time to drive AFTER the arm is
									// down
		double driveBeforeArm = 0.0; // This is the time to drive BEFORE the arm
										// is down

		// SCOTT - This drivingScalingFactor might need to be adjusted to affect
		// the
		// distance the robot travels after the arm is down
		double drivingScalingFactor = 1.2; // this is to affect the speed and
											// distance after the arm is down
		// set to 1.0 to match the last practice run

		driveBeforeArm = 1000.0;

		driveForTime = 3500;

		driveForTime = driveForTime - driveBeforeArm;

		driveForTime = driveForTime / drivingScalingFactor;

		System.out.println("driveForTime is " + driveForTime);

		System.out.println("driveBeforeArm is " + driveBeforeArm);

		double Kp = 0.03; // myGyro.getRate();
		double turningValue;
		int print_counter = 0;
		double drivingLoops = 0.0;

		// Timer autonomousTimer = new Timer();
		// autonomousTimer.start();

		/********************** Put the arm down *****************************/

		Timer armTimer = new Timer();
		armTimer.start();

		// This is where the adjustment for the arm will have to be made
		// to match the new design for the collector - SCOTT

		double rot_angle = 0.0;

		while (!(armTimer.hasPeriodPassed(0.7))) // number passed in is in
													// seconds
		{
			arm.setInverted(true);

			arm.set(0.5);

			armDrive = true;

			Timer.delay(0.005);

			rot_angle = pot.get();

			System.out.println("In loop!");

			SmartDashboard.putNumber("Arm Angle", rot_angle);

			if (rot_angle >= 45.0) {
				System.out.println("Potentiometer has reached: " + pot.get() + " degrees. Breaking out of loop");

				// break;
			}
		}

		arm.set(0);

		/********************** Drive Forward ***************************/

		while (drivingLoops < driveBeforeArm) // ||
												// !(armTimer.hasPeriodPassed(15)))
		{
			int encoder_count = 0;

			turningValue = -myGyro.getAngle() * Kp; // THIS WILL USE THE GYRO
													// INPUT TO DETERMINE
													// TURNING ANGLE

			myRobot.drive(-0.22, turningValue); // THIS IS GOING FORWARDS
												// (COLLECTOR IN FRONT)

			drivingLoops++;

			Timer.delay(0.002);
			print_counter++;
			if ((print_counter % 200) == 0) {
				//System.out.println("Current GYRO Angle: " + myGyro.getAngle());
			}

			if (print_counter % 50 == 0) {
				System.out.println("Distance: " + encoder.getDistance() + " Count: " + encoder.get() + " Raw: "
						+ encoder.getRaw());

				SmartDashboard.putNumber("Distance Traveled: ", encoder.getDistance());

				SmartDashboard.putNumber("Raw Count: ", encoder.get());

			}

			if (encoder.getDistance() >= requested_travel_distance) {
				// break; ///out of loop
			}
		}

		/********************** Put the arm down *****************************/

		armTimer.reset();
		armTimer.start();

		// This is where the adjustment for the arm will have to be made
		// to match the new design for the collector - SCOTT

		while (!(armTimer.hasPeriodPassed(2.2))) // number passed in is in
													// seconds
		{
			arm.setInverted(true);

			arm.set(0.5);

			armDrive = true;

			Timer.delay(0.005);

			rot_angle = pot.get();

			System.out.println("In loop!");

			SmartDashboard.putNumber("Arm Angle", rot_angle);

			if (rot_angle >= 45.0) {
				System.out.println("Potentiometer has reached: " + pot.get() + " degrees. Breaking out of loop");

				// break;
			}
		}

		arm.set(0);

		/********************** Move forward *****************************/

		print_counter = 0;
		drivingLoops = 0.0;

		/// reset encoder to get it back to 0
		encoder.reset();

		Timer autonomousTimer = new Timer();

		autonomousTimer.start();

		double driveSpeed = 0.22 * drivingScalingFactor * 2; // to change speed,
																// change
																// scaling
																// factor above

		while (isAutonomous()) // || !(armTimer.hasPeriodPassed(15)))
		{
			turningValue = -myGyro.getAngle() * Kp; // THIS WILL USE THE GYRO
													// INPUT TO DETERMINE
													// TURNING ANGLE
			// turningValue = 0.0; // COMMENT OUT THIS LINE IF USING THE GYRO
			// AND UNCOMMENT LINE ABOVE

			if (drivingLoops < driveForTime) {

				myRobot.drive(-driveSpeed, turningValue); // needs to go a
															// little faster

				drivingLoops++;
			}

			Timer.delay(0.002);

			print_counter++;

			if ((print_counter % 200) == 0) {
				// System.out.println("Current GYRO Angle: " +
				// myGyro.getAngle());
			}

			if (print_counter % 50 == 0) {
				System.out.println("Distance: " + encoder.getDistance() + " Count: " + encoder.get() + " Raw: "
						+ encoder.getRaw());

				SmartDashboard.putNumber("Distance Traveled: ", encoder.getDistance());

				SmartDashboard.putNumber("Raw Count: ", encoder.get());
			}

			if (encoder.getDistance() >= requested_travel_distance) {
				// break; ///out of loop
			}
		}
	}

	public void portcullis() {
		/// reset encoder
		double requested_travel_distance = 0.0;
		myGyro.reset();
		myGyro.setSensitivity(voltsPerDegreePerSecond);
		myGyro.calibrate(); // Added by SRH at 15:11
		double driveForTime = 0.0; // This is the time to drive AFTER the arm is
									// down
		double driveBeforeArm = 0.0; // This is the time to drive BEFORE the arm
										// is down

		// SCOTT - This drivingScalingFactor might need to be adjusted to affect
		// the
		// distance the robot travels after the arm is down
		double drivingScalingFactor = 1.2; // this is to affect the speed and
											// distance after the arm is down
		// set to 1.0 to match the last practice run

		driveBeforeArm = 1200.0;

		driveForTime = 3500;

		driveForTime = driveForTime - driveBeforeArm;

		driveForTime = driveForTime / drivingScalingFactor;

		System.out.println("driveForTime is " + driveForTime);

		System.out.println("driveBeforeArm is " + driveBeforeArm);

		double Kp = 0.03; // myGyro.getRate();
		double turningValue;
		int print_counter = 0;
		double drivingLoops = 0.0;

		// Timer autonomousTimer = new Timer();
		// autonomousTimer.start();

		/********************** Put the arm down *****************************/

		Timer armTimer = new Timer();
		armTimer.start();

		// This is where the adjustment for the arm will have to be made
		// to match the new design for the collector - SCOTT

		double rot_angle = 0.0;

		while (!(armTimer.hasPeriodPassed(2.7))) // number passed in is in
													// seconds
		{
			arm.setInverted(true);

			arm.set(0.5);

			armDrive = true;

			Timer.delay(0.005);

			rot_angle = pot.get();

			System.out.println("In loop!");

			SmartDashboard.putNumber("Arm Angle", rot_angle);

			if (rot_angle >= 45.0) {
				System.out.println("Potentiometer has reached: " + pot.get() + " degrees. Breaking out of loop");

				// break;
			}
		}

		arm.set(0);

		/********************** Drive Forward ***************************/

		while (drivingLoops < driveBeforeArm) // ||
												// !(armTimer.hasPeriodPassed(15)))
		{
			int encoder_count = 0;

			turningValue = -myGyro.getAngle() * Kp; // THIS WILL USE THE GYRO
													// INPUT TO DETERMINE
													// TURNING ANGLE

			myRobot.drive(-0.22, turningValue); // THIS IS GOING FORWARDS
												// (COLLECTOR IN FRONT)

			drivingLoops++;

			Timer.delay(0.002);
			print_counter++;
			if ((print_counter % 200) == 0) {
				//System.out.println("Current GYRO Angle: " + myGyro.getAngle());
			}

			if (print_counter % 50 == 0) {
				System.out.println("Distance: " + encoder.getDistance() + " Count: " + encoder.get() + " Raw: "
						+ encoder.getRaw());

				SmartDashboard.putNumber("Distance Traveled: ", encoder.getDistance());

				SmartDashboard.putNumber("Raw Count: ", encoder.get());

			}

			if (encoder.getDistance() >= requested_travel_distance) {
				// break; ///out of loop
			}
		}

		/********************** Put the arm up *****************************/
		armTimer.reset();

		armTimer.start();

		// This is where the adjustment for the arm will have to be made
		// to match the new design for the collector - SCOTT

		rot_angle = 0.0;

		double driveSpeed = 0.22 * drivingScalingFactor * 2; // to change speed,
																// change
																// scaling
																// factor above

		while (!(armTimer.hasPeriodPassed(2.0))) // number passed in is in
													// seconds
		{
			arm.setInverted(false);

			arm.set(0.5);

			armDrive = true;

			turningValue = -myGyro.getAngle() * Kp; // THIS WILL USE THE GYRO
													// INPUT TO DETERMINE
													// TURNING ANGLE

			myRobot.drive(-driveSpeed, turningValue); // needs to go a little
														// faster

			Timer.delay(0.005);

			rot_angle = pot.get();

			SmartDashboard.putNumber("Arm Angle", rot_angle);

			if (rot_angle >= 45.0) {
				System.out.println("Potentiometer has reached: " + pot.get() + " degrees. Breaking out of loop");

				// break;
			}
		}

		arm.set(0);

		/********************** Move forward *****************************/

		print_counter = 0;
		drivingLoops = 0.0;

		/// reset encoder to get it back to 0
		encoder.reset();

		Timer autonomousTimer = new Timer();

		autonomousTimer.start();

		while (isAutonomous()) // || !(armTimer.hasPeriodPassed(15)))
		{
			SmartDashboard.putNumber("Gyro value", myGyro.getAngle());
			turningValue = -myGyro.getAngle() * Kp; // THIS WILL USE THE GYRO
													// INPUT TO DETERMINE
													// TURNING ANGLE
			// turningValue = 0.0; // COMMENT OUT THIS LINE IF USING THE GYRO
			// AND UNCOMMENT LINE ABOVE

			if (drivingLoops < driveForTime) {

				myRobot.drive(-driveSpeed, turningValue); // needs to go a
															// little faster

				drivingLoops++;
			}

			Timer.delay(0.002);

			print_counter++;

			if ((print_counter % 200) == 0) {
				//System.out.println("Current GYRO Angle: " + myGyro.getAngle());
			}

			if (print_counter % 50 == 0) {
				System.out.println("Distance: " + encoder.getDistance() + " Count: " + encoder.get() + " Raw: "
						+ encoder.getRaw());

				SmartDashboard.putNumber("Distance Traveled: ", encoder.getDistance());

				SmartDashboard.putNumber("Raw Count: ", encoder.get());
			}

			if (encoder.getDistance() >= requested_travel_distance) {
				// break; ///out of loop
			}
		}
	}

	/**
	 * Runs the motors with arcade steering.
	 */
	public void operatorControl() {
		myRobot.setSafetyEnabled(true);
		int print_counter = 0;

		// myGyro.reset();
		// myGyro.setSensitivity(voltsPerDegreePerSecond);
		// myGyro.calibrate();
		// System.out.println("Gyro Angle after reset: " + myGyro.getAngle());
		//SmartDashboard.putString("1", "Hello World");
		if (true == SmartDashboard.getBoolean("DB/Button 0") && false == SmartDashboard.getBoolean("DB/Button 1")) {
			System.out.println("Button 0 is true");
			;
			SmartDashboard.putString("DB/String 0", "Cheval de Frise");
		} else if (false == SmartDashboard.getBoolean("DB/Button 0")
				&& true == SmartDashboard.getBoolean("DB/Button 1")) {
			System.out.println("Button 0 is false");
			;
			SmartDashboard.putString("DB/String 0", "Portcullis");
		} else {
			System.out.println("Button 0 is false");
			;
			SmartDashboard.putString("DB/String 0", "Low Bar");
		}

		while (isOperatorControl() && isEnabled()) {
			
			pully = false;

			armDrive = false;

			pullySpeed = 1.0;
			if(driver==CHIEBUKA)
			{
			myRobot.arcadeDrive(forward*stick.getRawAxis(5), -forward*stick.getRawAxis(4), true); // drive
																					// with
																					// arcade
																					// style
																					// (use
																					// right
																					// stick)
			// System.out.println("Moving foraward: " + temp + "," + temp1);
			Timer.delay(0.005); // wait for a motor update time
			}
			else if(driver == KOBY)
			{
				myRobot.tankDrive(stick.getRawAxis(1), stick.getRawAxis(5), true);	
				
			}
			else
			{
				System.out.println("Baby you can drive my car");
			}
			print_counter++;
			SmartDashboard.putNumber("Gyro", (int)((myGyro.getAngle())%360));
			if ((print_counter % 200) == 0) {
				//System.out.println("Current GYRO Angle: " + myGyro.getAngle());
			}

			if (stick.getRawAxis(2) > 0) // Left trigger
			{
				arm.setInverted(true);
				arm.set(stick.getRawAxis(2) / 2);
				armDrive = true;
			}
			if (stick.getRawAxis(3) > 0) // Right trigger
			{
				arm.setInverted(false);
				arm.set(stick.getRawAxis(3) / 2);
				armDrive = true;
			}
			if (armDrive == false) {
				arm.set(0);
			}

			if (stick.getRawButton(5)) { // Left bumper
				/*collector.setInverted(true);
				collector.set(pullySpeed);
				pully = true;*/
				turnSpeed=turnSpeed-.05;
				System.out.println(turnSpeed);
				/*if(turnSpeed<.50)
				{
					turnSpeed=.5;
				}*/
				SmartDashboard.putString("DB/String 5", "Turn Speed is: " + turnSpeed);	

			}
			if (stick.getRawButton(6)) { // Right bumper
				// System.out.println("right button pressed " + pullySpeed);
				/*collector.setInverted(false);
				collector.set(pullySpeed);
				pully = true;*/
				
			}

			if (pully == false) {
				collector.set(0);
			}

			if (stick.getRawButton(7)) // back button
			{

			}
			if (stick.getRawButton(8)) // start button
			{

			}
			if (stick.getRawButton(1)) // A button
			{
				forward = forward*-1;
			}
			if (stick.getRawButton(2)) // B button
			{
				turnSpeed=turnSpeed+.05;
				System.out.println("TurnSpeed: " + turnSpeed);
				/*if(turnSpeed>.80)
				{
					turnSpeed=.8;
				}*/
				
				SmartDashboard.putString("DB/String 5", "Turn Speed is: " + turnSpeed);
			}
			if (stick.getRawButton(3)) // X button
			{
				turnSpeed=turnSpeed-.05;
				/*if(turnSpeed<.5)
				{
					turnSpeed=.5;
				}*/
				System.out.println("TurnSpeed: " + turnSpeed);
				SmartDashboard.putString("DB/String 5", "Turn Speed is: " + turnSpeed);
			}
			if (stick.getRawButton(4)) // Y button
			{
				SmartDashboard.putNumber("Gyro", (int)((myGyro.getAngle())%360));
				if(SmartDashboard.getBoolean("DB/Button 3")){
					driver=CHIEBUKA;
					SmartDashboard.putString("DB/String 5", "CHIEBUKA is driving");
				}
				else
				{
					driver=KOBY;
					SmartDashboard.putString("DB/String 5", "Koby is driving");
				}
			}
		}
	}
	
	/**
	 * A method that will turn the robot a given amount of degrees.  This turn could be off by a small EPSILON
	 * @param angle
	 */
	public void turn(double angle)
	{
		double EPSILON = 1;
		double startHeading = myGyro.getAngle()%360;
		double goalHeading = (startHeading + angle) % 360;
		double current = myGyro.getAngle();
		current = current % 360; 
		int safteyCount=0;
		//System.out.println("AT: " + startHeading + "\t GOAL: " + goalHeading + "");
		do{
			current = myGyro.getAngle();
			current = current % 360; 
		myRobot.arcadeDrive(0, turnSpeed);
		Timer.delay(0.005);
		safteyCount++;
		if(safteyCount>=300)
		{
			System.out.println("Hit the saftery stop");
			break;
		}
		//System.out.print("Math.abs("+goalHeading+"-"+current+")>="+EPSILON);
		//System.out.println("\t\tMath.abs("+(goalHeading-current)+")>="+EPSILON);
		}while(Math.abs(goalHeading-current)>=EPSILON);
		System.out.print("Math.abs("+goalHeading+"-"+current+")>="+EPSILON);
		System.out.println("\t\tMath.abs("+(goalHeading-current)+")>="+EPSILON);
	}

	/**
	 * Runs during test mode
	 */
	public void test() {
		//camera = CameraServer.getInstance();
		//camera.setQuality(85);
		//camera.getQuality();
		//camera.startAutomaticCapture("cam1");
	}
}
