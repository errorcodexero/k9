#include <WPILib.h>
#include <OSAL/Synchronized.h>
#include <OSAL/Task.h>
#include "Tachometer.h"
#include "Logger.h"

const double minSpeed      = 1000.;
const double maxSpeed      = 3500.;
const double pidThreshold  = 0.80;
const double vbusThreshold = 0.60;
const double maxOutput     = 0.70;
const double defaultTop    = 1400.;
const double defaultBottom = 2850.;
const double defaultP      = 0.300;
const double defaultI      = 0.003;
const double defaultD      = 0.000;

// #define HAVE_COMPRESSOR
// #define HAVE_TOP_WHEEL
// #define HAVE_TOP_CAN1
// #define HAVE_TOP_PWM1
// #define HAVE_TOP_CAN2
#define HAVE_BOTTOM_WHEEL
// #define HAVE_BOTTOM_CAN1
// #define HAVE_BOTTOM_PWM1
#define HAVE_BOTTOM_CAN2
// #define HAVE_ARM
// #define HAVE_INJECTOR
// #define HAVE_EJECTOR
// #define HAVE_LEGS

class ShootyDogThing : public IterativeRobot
{
#ifdef HAVE_COMPRESSOR
    Compressor *compressor;
#endif
#ifdef HAVE_TOP_WHEEL
#ifdef HAVE_TOP_CAN1
    CANJaguar *topWheel1;
#endif
#ifdef HAVE_TOP_PWM1
    Victor *topWheel1;
#endif
#ifdef HAVE_TOP_CAN2
    CANJaguar *topWheel2;
#endif
    Tachometer *topTach;
#endif
#ifdef HAVE_BOTTOM_WHEEL
#ifdef HAVE_BOTTOM_CAN1
    CANJaguar *bottomWheel1;
#endif
#ifdef HAVE_BOTTOM_PWM1
    Victor *bottomWheel1;
#endif
#ifdef HAVE_BOTTOM_CAN2
    CANJaguar *bottomWheel2;
#endif
    Tachometer *bottomTach;
#endif
#ifdef HAVE_ARM
    DoubleSolenoid *arm;
#endif
#ifdef HAVE_INJECTOR
    DoubleSolenoid *injectorL, *injectorR;
#endif
#ifdef HAVE_EJECTOR
    Solenoid *ejector;
#endif
#ifdef HAVE_LEGS
    Solenoid *legs;
#endif
    DriverStation *ds;
    DriverStationEnhancedIO *eio;
    Joystick *gamepad;
    bool topPID;
    bool bottomPID;
    double kP, kI, kD;
    bool spinFastNow;
    double topSpeed, bottomSpeed;
    double topJagSpeed, bottomJagSpeed;
    double topTachSpeed, bottomTachSpeed;
    int report;
    int dump;

public:
    ShootyDogThing():
#ifdef HAVE_COMPRESSOR
	compressor(NULL),
#endif
#ifdef HAVE_TOP_WHEEL
#ifdef HAVE_TOP_CAN1
	topWheel1(NULL),
#endif
#ifdef HAVE_TOP_PWM1
	topWheel1(NULL),
#endif
#ifdef HAVE_TOP_CAN2
	topWheel2(NULL),
#endif
	topTach(NULL),
#endif
#ifdef HAVE_BOTTOM_WHEEL
#ifdef HAVE_BOTTOM_CAN1
	bottomWheel1(NULL),
#endif
#ifdef HAVE_BOTTOM_PWM1
	bottomWheel1(NULL),
#endif
#ifdef HAVE_BOTTOM_CAN2
	bottomWheel2(NULL),
#endif
	bottomTach(NULL),
#endif
#ifdef HAVE_ARM
	arm(NULL),
#endif
#ifdef HAVE_INJECTOR
	injectorL(NULL),
	injectorR(NULL),
#endif
#ifdef HAVE_EJECTOR
	ejector(NULL),
#endif
#ifdef HAVE_LEGS
	legs(NULL),
#endif
	ds(NULL),
	eio(NULL),
	gamepad(NULL),
	kP(defaultP),
	kI(defaultI),
	kD(defaultD),
	spinFastNow(false),
	topSpeed(defaultTop),
	bottomSpeed(defaultBottom),
	topJagSpeed(0.),
	bottomJagSpeed(0.),
	topTachSpeed(0.),
	bottomTachSpeed(0.),
	report(0),
	dump(0)
    {
printf(">>> ShootyDogThing\n");
	;
printf("<<< ShootyDogThing\n");
    }

    ~ShootyDogThing()
    {
printf(">>> ~ShootyDogThing\n");

	delete gamepad;
#ifdef HAVE_LEGS
	delete legs;
#endif
#ifdef HAVE_EJECTOR
	delete ejector;
#endif
#ifdef HAVE_INJECTOR
	delete injectorL;
	delete injectorR;
#endif
#ifdef HAVE_ARM
	delete arm;
#endif
#ifdef HAVE_BOTTOM_WHEEL
	delete bottomTach;
#ifdef HAVE_BOTTOM_CAN2
	delete bottomWheel2;
#endif
#ifdef HAVE_BOTTOM_PWM1
	delete bottomWheel1;
#endif
#ifdef HAVE_BOTTOM_CAN1
	delete bottomWheel1;
#endif
#endif
#ifdef HAVE_TOP_WHEEL
	delete topTach;
#ifdef HAVE_TOP_CAN2
	delete topWheel2;
#endif
#ifdef HAVE_TOP_PWM1
	delete topWheel1;
#endif
#ifdef HAVE_TOP_CAN1
	delete topWheel1;
#endif
#endif
#ifdef HAVE_COMPRESSOR
	delete compressor;
#endif

printf("<<< ~ShootyDogThing\n");
    }
    
    /**
     * Robot-wide initialization code should go here.
     * 
     * Use this method for default Robot-wide initialization which will
     * be called when the robot is first powered on.  It will be called exactly 1 time.
     */
    void RobotInit()
    {
printf(">>> RobotInit\n");

	LogInit();

#ifdef HAVE_COMPRESSOR
	compressor  = new Compressor(1, 1);
#endif

#ifdef HAVE_TOP_WHEEL
#ifdef HAVE_TOP_CAN1
	topWheel1    = new CANJaguar(1);
	topWheel1->SetSafetyEnabled(false);	// motor safety off while configuring
	topWheel1->SetSpeedReference( CANJaguar::kSpeedRef_Encoder );
	topWheel1->ConfigEncoderCodesPerRev( 1 );
#endif
#ifdef HAVE_TOP_PWM1
	topWheel1    = new Victor(1);
	topWheel1->SetSafetyEnabled(false);	// motor safety off while configuring
#endif
#ifdef HAVE_TOP_CAN2
	topWheel2    = new CANJaguar(2);
	topWheel2->SetSafetyEnabled(false);	// motor safety off while configuring
	topWheel2->SetSpeedReference( CANJaguar::kSpeedRef_Encoder );
	topWheel2->ConfigEncoderCodesPerRev( 1 );
#endif
	topTach      = new Tachometer(2);
#endif

#ifdef HAVE_BOTTOM_WHEEL
#ifdef HAVE_BOTTOM_CAN1
	bottomWheel1 = new CANJaguar(3);
	bottomWheel1->SetSafetyEnabled(false);	// motor safety off while configuring
	bottomWheel1->SetSpeedReference( CANJaguar::kSpeedRef_Encoder );
	bottomWheel1->ConfigEncoderCodesPerRev( 1 );
#endif
#ifdef HAVE_BOTTOM_PWM1
	bottomWheel1 = new Victor(2);
	bottomWheel1->SetSafetyEnabled(false);	// motor safety off while configuring
#endif
#ifdef HAVE_BOTTOM_CAN2
	bottomWheel2 = new CANJaguar(4);
	bottomWheel2->SetSafetyEnabled(false);	// motor safety off while configuring
	bottomWheel2->SetSpeedReference( CANJaguar::kSpeedRef_Encoder );
	bottomWheel2->ConfigEncoderCodesPerRev( 1 );
#endif
	bottomTach   = new Tachometer(3);
#endif

#ifdef HAVE_ARM
	arm          = new DoubleSolenoid(2, 1);
#endif
#ifdef HAVE_INJECTOR
	injectorL    = new DoubleSolenoid(5, 3);
	injectorR    = new DoubleSolenoid(6, 4);
#endif
#ifdef HAVE_EJECTOR
	ejector      = new Solenoid(7);
#endif
#ifdef HAVE_LEGS
	legs         = new Solenoid(8);
#endif

	ds           = DriverStation::GetInstance();
	eio          = &ds->GetEnhancedIO();
	gamepad      = new Joystick(1);

	LiveWindow *lw = LiveWindow::GetInstance();
#ifdef HAVE_COMPRESSOR
	lw->AddActuator("K9", "Compressor", compressor);
#endif
#ifdef HAVE_TOP_WHEEL
#ifdef HAVE_TOP_CAN1
	lw->AddActuator("K9", "Top1",       topWheel1);
#endif
#ifdef HAVE_TOP_PWM1
	lw->AddActuator("K9", "Top1",       topWheel1);
#endif
#ifdef HAVE_TOP_CAN2
	lw->AddActuator("K9", "Top2",       topWheel2);
#endif
#endif
#ifdef HAVE_BOTTOM_WHEEL
#ifdef HAVE_BOTTOM_CAN1
	lw->AddActuator("K9", "Bottom1",    bottomWheel1);
#endif
#ifdef HAVE_BOTTOM_PWM1
	lw->AddActuator("K9", "Bottom1",    bottomWheel1);
#endif
#ifdef HAVE_BOTTOM_CAN2
	lw->AddActuator("K9", "Bottom2",    bottomWheel2);
#endif
#endif
#ifdef HAVE_ARM
	lw->AddActuator("K9", "Arm",        arm);
#endif
#ifdef HAVE_INJECTOR
	lw->AddActuator("K9", "InjectorL",  injectorL);
	lw->AddActuator("K9", "InjectorR",  injectorR);
#endif
#ifdef HAVE_EJECTOR
	lw->AddActuator("K9", "Ejector",    ejector);
#endif
#ifdef HAVE_LEGS
	lw->AddActuator("K9", "Legs",       legs);
#endif

	SmartDashboard::PutNumber("Shooter P", kP);
	SmartDashboard::PutNumber("Shooter I", kI);
	SmartDashboard::PutNumber("Shooter D", kD);

	spinFastNow = false;

#ifdef HAVE_TOP_WHEEL
	SmartDashboard::PutNumber("Top Set      ", topSpeed);
#ifdef HAVE_TOP_CAN1
	SmartDashboard::PutNumber("Top Current 1", 0.0);
#endif
#ifdef HAVE_TOP_CAN2
	SmartDashboard::PutNumber("Top Current 2", 0.0);
	SmartDashboard::PutNumber("Top Jag      ", 0.0);
#endif
	SmartDashboard::PutNumber("Top Tach     ", 0.0);
#endif

#ifdef HAVE_BOTTOM_WHEEL
	SmartDashboard::PutNumber("Bottom Set      ", bottomSpeed);
#ifdef HAVE_BOTTOM_CAN1
	SmartDashboard::PutNumber("Bottom Current 1", 0.0);
#endif
#ifdef HAVE_BOTTOM_CAN2
	SmartDashboard::PutNumber("Bottom Current 2", 0.0);
	SmartDashboard::PutNumber("Bottom Jag      ", 0.0);
#endif
	SmartDashboard::PutNumber("Bottom Tach     ", 0.0);
#endif

	SetPeriod(0); 	//Set update period to sync with robot control packets (20ms nominal)

printf("<<< RobotInit\n");
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

    void StartWheels()
    {
	if (!spinFastNow) {
printf(">>> StartWheels\n");
	    Log(LOG_START, 0, 0);

	    spinFastNow = true;

	    // start shooter wheels in %vbus mode, max output
#ifdef HAVE_TOP_WHEEL
#ifdef HAVE_TOP_CAN1
	    jagVbus(topWheel1,    maxOutput);
	    Log(LOG_MODE, 1, 1);
#endif
#ifdef HAVE_TOP_PWM1
	    topWheel1->Set(maxOutput);
	    Log(LOG_MODE, 1, 1);
#endif
#ifdef HAVE_TOP_CAN2
	    jagVbus(topWheel2,    maxOutput);
	    Log(LOG_MODE, 2, 1);
#endif
#endif
#ifdef HAVE_BOTTOM_WHEEL
#ifdef HAVE_BOTTOM_CAN1
	    jagVbus(bottomWheel1, maxOutput);
	    Log(LOG_MODE, 3, 1);
#endif
#ifdef HAVE_BOTTOM_PWM1
	    bottomWheel1->Set(maxOutput);
	    Log(LOG_MODE, 3, 1);
#endif
#ifdef HAVE_BOTTOM_CAN2
	    jagVbus(bottomWheel2, maxOutput);
	    Log(LOG_MODE, 4, 1);
#endif
#endif
	    topPID = bottomPID = false;

	    // reset reporting counter
	    report = 0;
printf("<<< StartWheels\n");
	}
    }

    void StopWheels()
    {
	if (spinFastNow) {
printf(">>> StopWheels\n");
	    Log(LOG_STOP, 0, 0);

	    spinFastNow = false;

#ifdef HAVE_TOP_WHEEL
#ifdef HAVE_TOP_CAN1
	    jagStop(topWheel1);
	    Log(LOG_MODE, 1, 0);
#endif
#ifdef HAVE_TOP_PWM1
	    topWheel1->Disable();
	    Log(LOG_MODE, 1, 0);
#endif
#ifdef HAVE_TOP_CAN2
	    jagStop(topWheel2);
	    Log(LOG_MODE, 2, 0);
#endif
#endif
#ifdef HAVE_BOTTOM_WHEEL
#ifdef HAVE_BOTTOM_CAN1
	    jagStop(bottomWheel1);
	    Log(LOG_MODE, 3, 0);
#endif
#ifdef HAVE_BOTTOM_PWM1
	    bottomWheel1->Disable();
	    Log(LOG_MODE, 3, 0);
#endif
#ifdef HAVE_BOTTOM_CAN2
	    jagStop(bottomWheel2);
	    Log(LOG_MODE, 4, 0);
#endif
#endif

	    topPID = bottomPID = false;
printf("<<< StopWheels\n");
	}
    }

    void RunWheels()
    {
//	uint32_t t0, t1, t2, t3;

	// schedule updates to avoid overloading CAN bus or CPU
	switch (report++) {
	case 12:		// 240 milliseconds
	    report = 0;		// reset counter
	case 0:
	    // Update PID parameters
	    double newP = SmartDashboard::GetNumber("Shooter P");
	    double newI = SmartDashboard::GetNumber("Shooter I");
	    double newD = SmartDashboard::GetNumber("Shooter D");
	    if (newP != kP || newI != kI || newD != kD) {
		kP = newP;
		kI = newI;
		kD = newD;
#ifdef HAVE_TOP_WHEEL
		if (topPID) {
#ifdef HAVE_TOP_CAN1
		    ; // topWheel1->SetPID( kP, kI, kD );
#endif
#ifdef HAVE_TOP_CAN2
		    topWheel2->SetPID( kP, kI, kD );
#endif
		}
#endif
#ifdef HAVE_BOTTOM_WHEEL
		if (bottomPID) {
#ifdef HAVE_BOTTOM_CAN1
		    ; // bottomWheel1->SetPID( kP, kI, kD );
#endif
#ifdef HAVE_BOTTOM_CAN2
		    bottomWheel2->SetPID( kP, kI, kD );
#endif
		}
#endif
	    }
	    break;

	case 4:			// 80 milliseconds
#ifdef HAVE_TOP_WHEEL
//t0 = GetFPGATime();
	    // Get top output voltage, current and measured speed
#ifdef HAVE_TOP_CAN1
	    double topI1 = topWheel1->GetOutputCurrent();
#endif
#ifdef HAVE_TOP_CAN2
	    double topI2 = topWheel2->GetOutputCurrent();
	    topJagSpeed  = topWheel2->GetSpeed(); 
#endif
//t1 = GetFPGATime();
	    topTachSpeed = topTach->PIDGet();

#ifdef HAVE_TOP_CAN1
	    // stupid floating point!
	    Log(LOG_CURRENT, 1, (uint32_t)(topI1 * 1000 + 0.5));
#endif
#ifdef HAVE_TOP_CAN2
	    Log(LOG_CURRENT, 2, (uint32_t)(topI2 * 1000 + 0.5));
	    Log(LOG_SPEED,   2, (uint32_t)(topJagSpeed + 0.5));
#endif

	    // Send values to SmartDashboard
#ifdef HAVE_TOP_CAN1
	    SmartDashboard::PutNumber("Top Current 1", topI1);
#endif
#ifdef HAVE_TOP_CAN2
	    SmartDashboard::PutNumber("Top Current 2", topI2);
	    SmartDashboard::PutNumber("Top Jag      ", topJagSpeed);
#endif
	    SmartDashboard::PutNumber("Top Tach     ", topTachSpeed);

	    // Get setpoint
	    topSpeed = SmartDashboard::GetNumber("Top Set      ");
//t2 = GetFPGATime();

	    if (spinFastNow) {
		if (topPID) {
		    if (topJagSpeed < topSpeed * vbusThreshold) {
			topPID = false;
			// below threshold: switch both motors to full output
#ifdef HAVE_TOP_CAN1
			jagVbus(topWheel1, maxOutput);
			Log(LOG_MODE, 1, 1);
#endif
#ifdef HAVE_TOP_PWM1
			topWheel1->Set(maxOutput);
			Log(LOG_MODE, 1, 1);
#endif
#ifdef HAVE_TOP_CAN2
			jagVbus(topWheel2, maxOutput);
			Log(LOG_MODE, 2, 1);
#endif
		    } else {
			; // above threshold: run motor 1 off, PID on motor 2
#ifdef HAVE_TOP_CAN1
			topWheel1->Set(0.0);
#endif
#ifdef HAVE_TOP_PWM1
			topWheel1->Set(0.0);
#endif
#ifdef HAVE_TOP_CAN2
			topWheel2->Set(topSpeed);
#endif
		    }
		} else {
		    if (topJagSpeed >= topSpeed * pidThreshold) {
			; // above threshold: switch motor 1 off, motor 2 PID
			topPID = true;
#ifdef HAVE_TOP_CAN1
			topWheel1->Set(0.0);
#endif
#ifdef HAVE_TOP_PWM1
			topWheel1->Set(0.0);
#endif
#ifdef HAVE_TOP_CAN2
			jagPID(topWheel2, topSpeed);
			Log(LOG_MODE, 2, 2);
#endif
		    } else {
			; // below threshold: run both motors at full output
#ifdef HAVE_TOP_CAN1
			topWheel1->Set(maxOutput);
#endif
#ifdef HAVE_TOP_PWM1
			topWheel1->Set(maxOutput);
#endif
#ifdef HAVE_TOP_CAN2
			topWheel2->Set(maxOutput);
#endif
		    }
		}
//t3 = GetFPGATime();
//printf("%10u %10u %10u\n", (uint32_t)(t1 - t0), (uint32_t)(t2 - t1), (uint32_t)(t3 - t2));
	    }
#endif

	    break;

	case 8:		// 160 milliseconds
#ifdef HAVE_BOTTOM_WHEEL
	    // Get bottom output voltage, current and measured speed
//t0 = GetFPGATime();
#ifdef HAVE_BOTTOM_CAN1
	    double bottomI1 = bottomWheel1->GetOutputCurrent();
#endif
#ifdef HAVE_BOTTOM_CAN2
	    double bottomI2 = bottomWheel2->GetOutputCurrent();
	    bottomJagSpeed  = bottomWheel2->GetSpeed();
#endif
//t1 = GetFPGATime();
	    bottomTachSpeed = bottomTach->PIDGet();

#ifdef HAVE_BOTTOM_CAN1
	    Log(LOG_CURRENT, 3, (uint32_t)(bottomI1 * 1000 + 0.5));
#endif
#ifdef HAVE_BOTTOM_CAN2
	    Log(LOG_CURRENT, 4, (uint32_t)(bottomI2 * 1000 + 0.5));
	    Log(LOG_SPEED,   4, (uint32_t)(bottomJagSpeed + 0.5));
#endif

	    // Send values to SmartDashboard
#ifdef HAVE_BOTTOM_CAN1
	    SmartDashboard::PutNumber("Bottom Current 1", bottomI1);
#endif
#ifdef HAVE_BOTTOM_CAN2
	    SmartDashboard::PutNumber("Bottom Current 2", bottomI2);
	    SmartDashboard::PutNumber("Bottom Jag      ", bottomJagSpeed);
#endif
	    SmartDashboard::PutNumber("Bottom Tach     ", bottomTachSpeed);

	    // Get setpoint
	    bottomSpeed = SmartDashboard::GetNumber("Bottom Set      ");
//t2 = GetFPGATime();

	    if (spinFastNow) {
		if (bottomPID) {
		    if (bottomJagSpeed < bottomSpeed * vbusThreshold) {
			bottomPID = false;
			// below threshold: switch both motors to full output
#ifdef HAVE_BOTTOM_CAN1
			jagVbus(bottomWheel1, maxOutput);
			Log(LOG_MODE, 3, 1);
#endif
#ifdef HAVE_BOTTOM_PWM1
			bottomWheel1->Set(maxOutput);
			Log(LOG_MODE, 3, 1);
#endif
#ifdef HAVE_BOTTOM_CAN2
			jagVbus(bottomWheel2, maxOutput);
			Log(LOG_MODE, 4, 1);
#endif
		    } else {
			; // above threshold: run motor 1 off, PID on motor 2
#ifdef HAVE_BOTTOM_CAN1
			bottomWheel1->Set(0.0);
#endif
#ifdef HAVE_BOTTOM_PWM1
			bottomWheel1->Set(0.0);
#endif
#ifdef HAVE_BOTTOM_CAN2
			bottomWheel2->Set(bottomSpeed);
#endif
		    }
		} else {
		    if (bottomJagSpeed >= bottomSpeed * pidThreshold) {
			// above threshold: switch motor 1 off, motor 2 PID
			bottomPID = true;
#ifdef HAVE_BOTTOM_CAN1
			bottomWheel1->Set(0.0);
#endif
#ifdef HAVE_BOTTOM_PWM1
			bottomWheel1->Set(0.0);
#endif
#ifdef HAVE_BOTTOM_CAN2
			jagPID(bottomWheel2, bottomSpeed);
			Log(LOG_MODE, 4, 2);
#endif
		    } else {
			; // below threshold: run both motors at full output
#ifdef HAVE_BOTTOM_CAN1
			bottomWheel1->Set(maxOutput);
#endif
#ifdef HAVE_BOTTOM_PWM1
			bottomWheel1->Set(maxOutput);
#endif
#ifdef HAVE_BOTTOM_CAN2
			bottomWheel2->Set(maxOutput);
#endif
		    }
		}
//t3 = GetFPGATime();
//printf("%10u %10u %10u\n", (uint32_t)(t1 - t0), (uint32_t)(t2 - t1), (uint32_t)(t3 - t2));
	    }
#endif
	    break;
	}
    }

    /**
     * Initialization code for disabled mode should go here.
     * 
     * Use this method for initialization code which will be called each time
     * the robot enters disabled mode. 
     */
    void DisabledInit()
    {
printf(">>> DisabledInit\n");
	StopWheels();

#ifdef HAVE_ARM
	arm->Set(DoubleSolenoid::kOff);
#endif
#ifdef HAVE_INJECTOR
	injectorL->Set(DoubleSolenoid::kOff);
	injectorR->Set(DoubleSolenoid::kOff);
#endif
#ifdef HAVE_EJECTOR
	// ejector->Set(false);
#endif
#ifdef HAVE_LEGS
	// legs->Set(false);
#endif
#ifdef HAVE_COMPRESSOR
	compressor->Stop();
#endif
printf("<<< DisabledInit\n");
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
	RunWheels();

	// respond to log dump request even when disabled
	if (!eio->GetDigital(13))
	{
	    if (!dump)
	    {
		LogSave("/ni-rt/system/k9.csv");
	    }
	    dump = true;
	}
	else
	{
	    dump = false;
	}
    }

    /**
     * Initialization code for autonomous mode should go here.
     * 
     * Use this method for initialization code which will be called each time
     * the robot enters autonomous mode.
     */
    void AutonomousInit()
    {
printf(">>> AutonomousInit\n");

printf("<<< AutonomousInit\n");
    }

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
printf(">>> TeleopInit\n");
#ifdef HAVE_COMPRESSOR
	compressor->Start();
#endif
#ifdef HAVE_ARM
	arm->Set(DoubleSolenoid::kForward);
#endif
#ifdef HAVE_INJECTOR
	injectorL->Set(DoubleSolenoid::kReverse);
	injectorR->Set(DoubleSolenoid::kReverse);
#endif
#ifdef HAVE_EJECTOR
	ejector->Set(false);
#endif
#ifdef HAVE_LEGS
	// legs->Set(true);
#endif

	// StartWheels();
printf("<<< TeleopInit\n");
    }


    /**
     * Periodic code for teleop mode should go here.
     *
     * Use this method for code which will be called periodically at a regular
     * rate while the robot is in teleop mode.
     */
    void TeleopPeriodic()
    {
	if (!eio->GetDigital(1))
	{
	    StartWheels();
	}
	else if (!eio->GetDigital(2))
	{
	    StopWheels();
	}

	RunWheels();

#ifdef HAVE_LEGS
	if (!eio->GetDigital(3))
	{
	    legs->Set(true);
	}
	else if (!eio->GetDigital(4))
	{
	    legs->Set(false);
	}
#endif

#ifdef HAVE_INJECTOR
	if (!eio->GetDigital(5))
	{
	    injectorL->Set(DoubleSolenoid::kForward);
	    injectorR->Set(DoubleSolenoid::kForward);
	}
	else if (!eio->GetDigital(6))
	{
	    injectorL->Set(DoubleSolenoid::kReverse);
	    injectorR->Set(DoubleSolenoid::kReverse);
	}
	else
	{
	    injectorL->Set(DoubleSolenoid::kOff);
	    injectorR->Set(DoubleSolenoid::kOff);
	}
#endif

#ifdef HAVE_EJECTOR
	if (!eio->GetDigital(7))
	{
	    ejector->Set(true);
	}
	else if (!eio->GetDigital(8))
	{
	    ejector->Set(false);
	}
#endif

	if (!eio->GetDigital(13))
	{
	    if (!dump)
	    {
		LogSave("/ni-rt/system/k9.csv");
	    }
	    dump = true;
	}
	else
	{
	    dump = false;
	}
    }

    /**
     * Initialization code for test mode should go here.
     * 
     * Use this method for initialization code which will be called each time
     * the robot enters test mode.
     */
    void TestInit()
    {
printf(">>> TestInit\n");
#ifdef HAVE_COMPRESSOR
	compressor->Start();
#endif
#ifdef HAVE_ARM
	arm->Set(DoubleSolenoid::kOff);
#endif
#ifdef HAVE_INJECTOR
	injectorL->Set(DoubleSolenoid::kOff);
	injectorR->Set(DoubleSolenoid::kOff);
#endif
#ifdef HAVE_EJECTOR
	ejector->Set(false);
#endif
#ifdef HAVE_LEGS
	legs->Set(false);
#endif

#ifdef HAVE_TOP_WHEEL
#ifdef HAVE_TOP_CAN1
	jagVbus(topWheel1, 0.0);
#endif
#ifdef HAVE_TOP_PWM1
	topWheel1->Set(0.0);
#endif
#ifdef HAVE_TOP_CAN2
	jagVbus(topWheel2, 0.0);
#endif
#endif
#ifdef HAVE_BOTTOM_WHEEL
#ifdef HAVE_BOTTOM_CAN1
	jagVbus(bottomWheel1, 0.0);
#endif
#ifdef HAVE_BOTTOM_PWM1
	bottomWheel1->Set(0.0);
#endif
#ifdef HAVE_BOTTOM_CAN2
	jagVbus(bottomWheel2, 0.0);
#endif
#endif
printf("<<< TestInit\n");
    }

    /**
     * Periodic code for test mode should go here.
     *
     * Use this method for code which will be called periodically at a regular
     * rate while the robot is in test mode.
     */
    void TestPeriodic() { }

};

START_ROBOT_CLASS(ShootyDogThing);

