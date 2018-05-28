#include <mbed.h>
#include <vector>
#include "source/EncodedMotor.h"	// motor encoder
#include "source/DebugMonitor.h"	// enable LCD2004 as debug monitor
#include <TextLCD.h>
#include "source/ShiftReg7Seg.h"
#include "source/MotorControl.h"
#include "source/EventVariable.h"
#include "source/MovingAverage.h"

// set baudrate at mbed_config.h default 115200
// I2C scanner included, derived from Arduino I2C scanner
// http://www.gammon.com.au/forum/?id=10896
//// Define constants
// volatile bool motorStartBtnChange = false;		// Start motor flag
volatile bool prevMotorSteady = false;		// Store previous motor steady state
//volatile float currentSpeed;
float refSpeedFloat = 0.0f;					// Floating value of reference speed
const float motor1RPM = 24.0f/50.0f;		// Motor rated RPM at 24RPM/50gear

/////////////////////////////////
//// Declare connection//////////
/////////////////////////////////
// L298N Control PinName
PinName MotorEnable = PA_15;		// connect ENA to PA_15
PinName MotorDirection1 = PA_14;	// connect IN1 to PA_14
PinName MotorDirection2 = PA_13;	// connect IN2 to PA_13
DigitalOut TorchEnable(PB_7);		// connect ENB to PB_7
// User Input PinName
PinName WeldStartStop = PA_11; 		// connect WeldStartStop Btn to PA_11
PinName MotorStartStop = PA_12;		// connect MotorStartStop Btn to PA_12
PinName MotorChangeDirection = PC_13;   // user btn on board, for changing motor direction
PinName knob = PA_4;				// connect Potential Meter to PA_4 (A2)
// Motor Encoder PinName
PinName MotorEncoderA = PA_9;		// connect MotorEncoderA (blue) to PA_9 (D8)
PinName MotorEncoderB = PA_8;		// connect MotorEncoderB (purple) to PA_8 (D7)

// LED Indicator Port
DigitalOut MotorLED(PB_15);		// connect MotorLED to PB_15
DigitalOut TorchLED(PB_14);		// connect TorchLED to PB_14
// Port Declaration
AnalogIn refSpeed(knob);			// Reference Speed from user through potentiometer
PwmOut MotorEnablePin(MotorEnable);
DigitalOut MotorDirectionPin1(MotorDirection1);
DigitalOut MotorDirectionPin2(MotorDirection2);

//// Initiate object
RawSerial pc(SERIAL_TX, SERIAL_RX, 115200);		                		// serial communication protocol
std::shared_ptr<EncodedMotor> encoder = std::make_shared<EncodedMotor>
        (MotorEncoderA, MotorEncoderB, 1848*4*50, 10, EncodeType::X4); 	// Encoded Motor object
std::unique_ptr<MotorControl> motor1 = std::make_unique<MotorControl>
        (MotorEnable, MotorDirection1, MotorDirection2, encoder, 0.20, 0.005, 0.08, motor1RPM);		// motor controller object, Kp, Ki, Kd specified
DebugMonitor debugger(&refSpeed, encoder, &pc);		        		// update status through LCD2004 and Serial Monitor
ShiftReg7Seg disp1(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS, 4, D9); 	// 7 segments display

//// Declare interrupt
Ticker statusUpdater;			// Periodic Interrupt for debugging purpose - disabled in testing
Ticker motorBlinkLEDTicker;			// LED Blinker
Ticker buttonRestart;          		// To cancel out fluctuating button signal
Ticker torchButtonRestart;          // To cancel out fluctuating button signal
InterruptIn motorBtn(MotorStartStop);		// Motor Button Interrupt
InterruptIn weldingBtn(WeldStartStop);		// Welding Button Interrupt
InterruptIn motorChgDirBtn(MotorChangeDirection);   // Motor Change Direction Interrupt (btn on board)

//// Declare thread
//Thread motorLEDBlinking;			// Thread to perform LED Blinking - disabled (possible unused variable)
Thread statusUpdateThread;      	// Thread to perform Status Update
Thread dispThread;              	// Thread to display 7 segments display

//// Declare event flag
EventFlags statusUpdateFlag;
//EventFlags interuptRestartFlag;	// disabled (possible unused variable)

//// Fwd declare
// void I2C_scan(); // disabled in testing
void motorStartBtnChangeEvent(bool &);		// Determine motor start status
void torchStartBtnChangeEvent(bool &);		// Determine motor start status
void motorRunner(); 
void MotorLEDBlinker(bool&);
void restartMotorButton();
void restartTorchButton();

// Initiate EventVariable
EventVariable<bool> motorStartBtnChange(false, &motorStartBtnChangeEvent);
EventVariable<bool> motorSteadySignal(true, &MotorLEDBlinker);
EventVariable<bool> weldSignal(false,&torchStartBtnChangeEvent);

// Testing Timer
Timer myTimer;

//// Define function
void statusUpdateEvent() {
    while(1)
    {
        statusUpdateFlag.wait_all(0x1);
/*        // Output status
        debugger.printSignal();

        // Output Flags to Serial monitor
        pc.printf("motorStartBtnChange: %d\n motorSteadySignal: %d\n weldSignal: %d\n TorchEnable = %d\n",
                  motorStartBtnChange.value, motorSteadySignal.value, weldSignal.value, TorchEnable.read());
        pc.printf("RefSpeed: %f\n Compensate: %f\n Speed: %f\n Error: %lf\n AdjError: %lf\n Current Direction: %d\n",
                  refSpeedFloat*100, motor1->readComp(), motor1->readSpeed(), motor1->readError(), motor1->readAdjError(),
                  motor1->getCurrentDirection());
        pc.printf( "Steady Count: %d\n",  motor1->getSteadyCount());*/ // disabled in testing

        pc.printf("%.3f, %.3f, %.3f \n", myTimer.read(), motor1->readSpeed(), motor1->readRefVolt()*100.0f); // time (s), rotation speed (min/rotation), refSpeed (min/rotation)
    }
}
void motorStartBtnChangeEvent(bool &motorState) {
	// Active-Deactivate Motor Rotation
	if (!motorState) {
		motorBlinkLEDTicker.detach();
		MotorLED = 0;
	}
	else {
		motor1->setRefVolt(refSpeedFloat);
		motorSteadySignal = false;
	}

	// disable motor button till next MotorLED blink .... to avoid noisy signal
	motorBtn.disable_irq();         // disable interrupt
	buttonRestart.attach(&restartMotorButton,1.0f);
}
void torchStartBtnChangeEvent(bool &torchState) {
    if(torchState){
        TorchLED = 1;
        TorchEnable = 1;
        motorBtn.disable_irq();
    }
    else{
        TorchLED = 0;
        TorchEnable = 0;
        motorBtn.enable_irq();
    }
    weldingBtn.disable_irq();
    torchButtonRestart.attach(&restartTorchButton,1.0f);
}
void motorRunner() {
    bool tempMotorSteady = motor1->run();			// run motor1 and read motor1 steady state
	if (tempMotorSteady != prevMotorSteady) motorSteadySignal = tempMotorSteady;
	prevMotorSteady = tempMotorSteady;
}
/*void motorStopper()
{
	motorSteadySignal = false;
	motor1->stop();
}*/ // disabled in testing
void MotorLEDBlinker(bool& motorSteady){
	if (motorSteady) { motorBlinkLEDTicker.detach(); MotorLED = 1; }
	else motorBlinkLEDTicker.attach([]() {MotorLED = !MotorLED; }, 0.5f);
}	// Run motor and set motorOnLED to blinking / solid light
void displayCurrentSpeed(){
    while(1){
    	if(motorStartBtnChange.value){
    		// motorStartBtnChange = 1 (i.e. motor started)
    		disp1.display(1/(motor1->readRefRPM()));
    	}
    	else{
			disp1.display(1/(refSpeedFloat*motor1RPM));
    	}
//        disp1.display(refSpeedFloat*100);
        wait(0.1);
    }
}
void restartMotorButton() {motorBtn.enable_irq(); buttonRestart.detach(); }
void restartTorchButton() {weldingBtn.enable_irq(); torchButtonRestart.detach(); }

int main() {
	pc.printf("Initiating\n");

	// Initiate Interrupt and Ticker
	//motorBtn.rise([]() {motorStartBtnChange = !motorStartBtnChange; });		// motorBtn OnChange - disabled in testing
	weldingBtn.rise([&]() {
		bool toSolenoid = motorStartBtnChange.value && !weldSignal;
		toSolenoid ? weldSignal = true : weldSignal = false; });			// weldingBtn OnChange
    motorChgDirBtn.rise([](){motor1->chgDirection(); });					// motorChgDirBtn OnChange
	statusUpdater.attach([](){statusUpdateFlag.set(0x1); }, 0.1f);		// periodic status update via flag - disabled in testing

    // Start Thread
	dispThread.start(displayCurrentSpeed);			// 7-segment Thread Start
    statusUpdateThread.start(&statusUpdateEvent);   // Start Status Update Event - disabled in testing

    // Initialize Output
    TorchLED = 0; 								// Initialize TorchLED

	pc.printf("Ready\n");

	// Testing sequence
    bool testingReset = false;
    motorBtn.rise([&testingReset](){testingReset = true; });

	wait(1);
    motor1->setRefVolt(refSpeed.read() *0.86f + 0.145f);
    myTimer.start();
	while(1){
/*		if (myTimer.read() > 30.0f) {motor1->stop(); }
		else {motorRunner(); }*/
        motorRunner();
        if (testingReset == true) {myTimer.reset(); testingReset = false; }
    }

/*	while (1) {
	    refSpeedFloat = refSpeed.read() *0.86f + 0.145f;
		if (motorStartBtnChange.value) {motorRunner();}
		else { motorStopper(); }
	}*/ // disabled in testing
}