#include "WPILib.h"

const double minSpeed = 1000.;
const double maxSpeed = 3500.;
const double pidThreshold = 0.80;
const double vbusThreshold = 0.60;


class ShootyDogThing : public IterativeRobot
{
    Compressor *compressor;
    CANJaguar *topWheel1, *topWheel2;
    CANJaguar *bottomWheel1, *bottomWheel2;
    DoubleSolenoid *arm;
    DoubleSolenoid *injectorL, *injectorR;
    Solenoid *ejector;
    Solenoid *legs;
    Joystick *gamepad;
    bool topPID;
    bool bottomPID;
    double kP, kI, kD;
    double topSpeed, bottomSpeed;
    double topMeasured, bottomMeasured;
    int report;

public:
    ShootyDogThing():
	compressor(NULL),
	topWheel1(NULL),
	topWheel2(NULL),
	bottomWheel1(NULL),
	bottomWheel2(NULL),
	arm(NULL),
	injectorL(NULL),
	injectorR(NULL),
	ejector(NULL),
	legs(NULL),
	gamepad(NULL),
	kP(1.000),
	kI(0.005),
	kD(0.000),
	topSpeed(1400.),
	bottomSpeed(2850.),
	report(0)
    {
	this->SetPeriod(0); 	//Set update period to sync with robot control packets (20ms nominal)
    }

    ~ShootyDogThing()
    {
	delete gamepad;
	delete legs;
	delete ejector;
	delete injectorL;
	delete injectorR;
	delete arm;
	delete bottomWheel2;
	delete bottomWheel1;
	delete topWheel2;
	delete topWheel1;
	delete compressor;
    }
    
    /**
     * Robot-wide initialization code should go here.
     * 
     * Use this method for default Robot-wide initialization which will
     * be called when the robot is first powered on.  It will be called exactly 1 time.
     */
    void RobotInit() {

	compressor  = new Compressor(1, 1);

	topWheel1    = new CANJaguar(1);
	topWheel1->SetSafetyEnabled(false);	// motor safety off while configuring
	topWheel1->SetSpeedReference( CANJaguar::kSpeedRef_Encoder );
	topWheel1->ConfigEncoderCodesPerRev( 1 );

	topWheel2    = new CANJaguar(2);
	topWheel2->SetSafetyEnabled(false);	// motor safety off while configuring
	topWheel2->SetSpeedReference( CANJaguar::kSpeedRef_Encoder );
	topWheel2->ConfigEncoderCodesPerRev( 1 );

	bottomWheel1 = new CANJaguar(3);
	bottomWheel1->SetSafetyEnabled(false);	// motor safety off while configuring
	bottomWheel1->SetSpeedReference( CANJaguar::kSpeedRef_Encoder );
	bottomWheel1->ConfigEncoderCodesPerRev( 1 );

	bottomWheel2 = new CANJaguar(4);
	bottomWheel2->SetSafetyEnabled(false);	// motor safety off while configuring
	bottomWheel2->SetSpeedReference( CANJaguar::kSpeedRef_Encoder );
	bottomWheel2->ConfigEncoderCodesPerRev( 1 );

	arm          = new DoubleSolenoid(2, 1);
	injectorL    = new DoubleSolenoid(5, 3);
	injectorR    = new DoubleSolenoid(6, 4);
	ejector      = new Solenoid(7);
	legs         = new Solenoid(8);

	gamepad      = new Joystick(1);

	LiveWindow *lw = LiveWindow::GetInstance();
	lw->AddActuator("K9", "Compressor", compressor);
	lw->AddActuator("K9", "Top1",       topWheel1);
	lw->AddActuator("K9", "Top2",       topWheel2);
	lw->AddActuator("K9", "Bottom1",    bottomWheel1);
	lw->AddActuator("K9", "Bottom2",    bottomWheel2);
	lw->AddActuator("K9", "Arm",        arm);
	lw->AddActuator("K9", "InjectorL",  injectorL);
	lw->AddActuator("K9", "InjectorR",  injectorR);
	lw->AddActuator("K9", "Ejector",    ejector);
	lw->AddActuator("K9", "Legs",       legs);

	SmartDashboard::PutNumber("Shooter P", kP);
	SmartDashboard::PutNumber("Shooter I", kI);
	SmartDashboard::PutNumber("Shooter D", kD);

	SmartDashboard::PutNumber("Top Speed",     topSpeed);
	SmartDashboard::PutNumber("Top Current 1", 0.0);
	SmartDashboard::PutNumber("Top Current 2", 0.0);
	SmartDashboard::PutNumber("Top Measured",  0.0);

	SmartDashboard::PutNumber("Bottom Speed",     bottomSpeed);
	SmartDashboard::PutNumber("Bottom Current 1", 0.0);
	SmartDashboard::PutNumber("Bottom Current 2", 0.0);
	SmartDashboard::PutNumber("Bottom Measured",  0.0);
    }

    // put jag in PID control mode, enabled
    void jagPID( CANJaguar *jag, double setpoint )
    {
	jag->ChangeControlMode( CANJaguar::kSpeed );
	jag->SetPID( kP, kI, kD );
	jag->EnableControl();
	jag->SetExpiration(2.0);
	jag->Set(setpoint, 0);
	jag->SetSafetyEnabled(true);
    }

    // put Jag in %vbus control mode, enabled
    void jagVbus( CANJaguar *jag, double setpoint )
    {
	jag->ChangeControlMode( CANJaguar::kPercentVbus );
	jag->EnableControl();
	jag->SetExpiration(2.0);
	jag->Set(setpoint, 0);
	jag->SetSafetyEnabled(true);
    }

    // put Jag in %vbus control mode, disabled
    void jagStop( CANJaguar *jag )
    {
	jag->Set(0.0, 0);
	jag->DisableControl();
	jag->SetSafetyEnabled(false);
    }

    /**
     * Initialization code for disabled mode should go here.
     * 
     * Use this method for initialization code which will be called each time
     * the robot enters disabled mode. 
     */
    void DisabledInit()
    {
	jagStop(topWheel1);
	jagStop(topWheel2);
	jagStop(bottomWheel1);
	jagStop(bottomWheel2);

	topPID = bottomPID = false;

	arm->Set(DoubleSolenoid::kOff);
	injectorL->Set(DoubleSolenoid::kOff);
	injectorR->Set(DoubleSolenoid::kOff);
	// ejector->Set(false);
	// legs->Set(false);

	compressor->Stop();
    }

    /**
     * Periodic code for disabled mode should go here.
     * 
     * Use this method for code which will be called periodically at a regular
     * rate while the robot is in disabled mode.
     */
    void DisabledPeriodic()
    {
	// Keep watching wheel speeds during spin-down
	// schedule updates to avoid overloading CAN bus or CPU
	switch (report++) {
	case 0:
	    break;

	case 8:			// 160 milliseconds
	    // Get top output voltage, current and measured speed
	    double topI1 = topWheel1->GetOutputCurrent();
	    double topI2 = topWheel2->GetOutputCurrent();
	    topMeasured = topWheel2->GetSpeed(); 

	    // Send values to SmartDashboard
	    SmartDashboard::PutNumber("Top Current 1", topI1);
	    SmartDashboard::PutNumber("Top Current 2", topI2);
	    SmartDashboard::PutNumber("Top Measured",  topMeasured);

	    break;

	case 16:		// 320 milliseconds
	    // Get bottom output voltage, current and measured speed
	    double bottomI1 = bottomWheel1->GetOutputCurrent();
	    double bottomI2 = bottomWheel2->GetOutputCurrent();
	    bottomMeasured = bottomWheel2->GetSpeed(); 

	    // Send values to SmartDashboard
	    SmartDashboard::PutNumber("Bottom Current 1", bottomI1);
	    SmartDashboard::PutNumber("Bottom Current 2", bottomI2);
	    SmartDashboard::PutNumber("Bottom Measured",  bottomMeasured);

	    break;

	case 24:		// 480 milliseconds
	    report = 0;		// reset counter
	    break;
	}
    }

    /**
     * Initialization code for autonomous mode should go here.
     * 
     * Use this method for initialization code which will be called each time
     * the robot enters autonomous mode.
     */
    void AutonomousInit() { }

    /**
     * Periodic code for autonomous mode should go here.
     *
     * Use this method for code which will be called periodically at a regular
     * rate while the robot is in autonomous mode.
     */
    void AutonomousPeriodic() { }

    /**
     * Initialization code for teleop mode should go here.
     * 
     * Use this method for initialization code which will be called each time
     * the robot enters teleop mode.
     */
    void TeleopInit()
    {
	compressor->Start();
	arm->Set(DoubleSolenoid::kForward);
	injectorL->Set(DoubleSolenoid::kReverse);
	injectorR->Set(DoubleSolenoid::kReverse);
	ejector->Set(false);
	legs->Set(true);

	// start shooter wheels in %vbus mode, full output
	jagVbus(topWheel1,    1.0);
	jagVbus(topWheel2,    1.0);
	jagVbus(bottomWheel1, 1.0);
	jagVbus(bottomWheel2, 1.0);
	topPID = bottomPID = false;

	// reset reporting counter
	report = 0;
    }


    /**
     * Periodic code for teleop mode should go here.
     *
     * Use this method for code which will be called periodically at a regular
     * rate while the robot is in teleop mode.
     */
    void ShootyDogThing::TeleopPeriodic() {
	// schedule updates to avoid overloading CAN bus or CPU
	switch (report++) {
	case 0:
	    // Update PID parameters
	    double newP = SmartDashboard::GetNumber("Shooter P");
	    double newI = SmartDashboard::GetNumber("Shooter I");
	    double newD = SmartDashboard::GetNumber("Shooter D");
	    if (newP != kP || newI != kI || newD != kD) {
		kP = newP;
		kI = newI;
		kD = newD;
		topWheel1->SetPID( kP, kI, kD );
		topWheel2->SetPID( kP, kI, kD );
		bottomWheel1->SetPID( kP, kI, kD );
		bottomWheel2->SetPID( kP, kI, kD );
	    }
	    break;

	case 8:			// 160 milliseconds
	    // Get top output voltage, current and measured speed
	    double topI1 = topWheel1->GetOutputCurrent();
	    double topI2 = topWheel2->GetOutputCurrent();
	    topMeasured = topWheel2->GetSpeed(); 

	    // Send values to SmartDashboard
	    SmartDashboard::PutNumber("Top Current 1", topI1);
	    SmartDashboard::PutNumber("Top Current 2", topI2);
	    SmartDashboard::PutNumber("Top Measured",  topMeasured);

	    topSpeed = SmartDashboard::GetNumber("Top Speed");

	    if (topPID) {
		if (topMeasured < topSpeed * vbusThreshold) {
		    topPID = false;
		    // below threshold: switch both motors to full output
		    jagVbus(topWheel1, 1.0);
		    jagVbus(topWheel2, 1.0);
		} else {
		    // above threshold: run motor 1 off, PID on motor 2
		    topWheel1->Set(0.0);
		    topWheel2->Set(topSpeed);
		}
	    } else {
		if (topMeasured >= topSpeed * pidThreshold) {
		    // above threshold: switch motor 1 off, motor 2 PID
		    topPID = true;
		    topWheel1->Set(0.0);
		    jagPID(topWheel2, topSpeed);
		} else {
		    // below threshold: run both motors at full output
		    topWheel1->Set(1.0);
		    topWheel2->Set(1.0);
		}
	    }

	    break;

	case 16:		// 320 milliseconds
	    // Get bottom output voltage, current and measured speed
	    double bottomI1 = bottomWheel1->GetOutputCurrent();
	    double bottomI2 = bottomWheel2->GetOutputCurrent();
	    bottomMeasured = bottomWheel2->GetSpeed(); 

	    // Send values to SmartDashboard
	    SmartDashboard::PutNumber("Bottom Current 1", bottomI1);
	    SmartDashboard::PutNumber("Bottom Current 2", bottomI2);
	    SmartDashboard::PutNumber("Bottom Measured",  bottomMeasured);

	    bottomSpeed = SmartDashboard::GetNumber("Bottom Speed");

	    if (bottomPID) {
		if (bottomMeasured < bottomSpeed * vbusThreshold) {
		    bottomPID = false;
		    // below threshold: switch both motors to full output
		    jagVbus(bottomWheel1, 1.0);
		    jagVbus(bottomWheel2, 1.0);
		} else {
		    // above threshold: run motor 1 off, PID on motor 2
		    bottomWheel1->Set(0.0);
		    bottomWheel2->Set(bottomSpeed);
		}
	    } else {
		if (bottomMeasured >= bottomSpeed * pidThreshold) {
		    // above threshold: switch motor 1 off, motor 2 PID
		    bottomPID = true;
		    bottomWheel1->Set(0.0);
		    jagPID(bottomWheel2, bottomSpeed);
		} else {
		    // below threshold: run both motors at full output
		    bottomWheel1->Set(1.0);
		    bottomWheel2->Set(1.0);
		}
	    }

	    break;

	case 24:		// 480 milliseconds
	    report = 0;		// reset counter
	    break;
	}

	if (gamepad->GetRawButton(4))
	{
	    injectorL->Set(DoubleSolenoid::kForward);
	    injectorR->Set(DoubleSolenoid::kForward);
	}
	else if (gamepad->GetRawButton(2))
	{
	    injectorL->Set(DoubleSolenoid::kReverse);
	    injectorR->Set(DoubleSolenoid::kReverse);
	}
	else
	{
	    injectorL->Set(DoubleSolenoid::kOff);
	    injectorR->Set(DoubleSolenoid::kOff);
	}

    }

    /**
     * Initialization code for test mode should go here.
     * 
     * Use this method for initialization code which will be called each time
     * the robot enters test mode.
     */
    void ShootyDogThing::TestInit()
    {
	compressor->Start();
	arm->Set(DoubleSolenoid::kOff);
	injectorL->Set(DoubleSolenoid::kOff);
	injectorR->Set(DoubleSolenoid::kOff);
	ejector->Set(false);
	legs->Set(false);

	jagVbus(topWheel1, 0.0);
	jagVbus(topWheel2, 0.0);
	jagVbus(bottomWheel1, 0.0);
	jagVbus(bottomWheel2, 0.0);
    }

    /**
     * Periodic code for test mode should go here.
     *
     * Use this method for code which will be called periodically at a regular
     * rate while the robot is in test mode.
     */
    void ShootyDogThing::TestPeriodic() { }

};

START_ROBOT_CLASS(ShootyDogThing);

