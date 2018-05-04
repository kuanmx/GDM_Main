#include "DebugMonitor.h"
#include "EncodedMotor.h"
#include "mbed_stats.h" // resource monitor for heap
#include "cmsis_os.h"   // resource monitor for stack

DebugMonitor::DebugMonitor(AnalogIn* knobPin, EncodedMotor* motorPtr, RawSerial* rawSerialPtr,
                           PinName I2C1_SDA, PinName I2C1_SDL, uint16_t lcdAddr, TextLCD::LCDType lcdtype, bool resourceEnabled) :
	i2c(I2C1_SDA, I2C1_SDL), lcd(&i2c, lcdAddr << 1, lcdtype),
	_knob(knobPin), _motorPtr(motorPtr), _rawSerialPtr(rawSerialPtr) , _resourceEnabled(resourceEnabled)
{
	lcd.cls();
	lcd.setMode(TextLCD_Base::LCDMode::DispOn);
	lcd.setBacklight(TextLCD::LCDBacklight::LightOn);

	// LCD Initialize
	lcd.printf("Initialize... ");
	wait(1.0f);
	lcd.cls();

    if (_resourceEnabled)
    {
        mbed_stats_heap_t heap_stats;
        osThreadId main_id = osThreadGetId();
    }
}

void DebugMonitor::printSignal() {
	_speedData = _motorPtr->getSpeed();
	_speed = std::get<0>(_speedData);
	_timeDiff = std::get<1>(_speedData); 
	
/*	//// Output to LCD2004
	// analogWrite value
	lcd.locate(0, 0);
	lcd.printf("PWM - %f", signal);
	// equivalent PWM voltage
	lcd.locate(0, 1);
	const float Vs_max = 10.5f;
	lcd.printf("Out Volt - %f", signal * Vs_max);
	// motor RPM
	lcd.locate(0, 2);
	lcd.printf("Motor RPM - %lf", _speed); */

	//// Output to Serial monitor
	_rawSerialPtr->printf("---\n");
	_rawSerialPtr->printf("refSpeed: %f\n Motor RPM: %f\n TimeDiff(us): %llu\n",
		_knob->read()*24.0f, _speed, _timeDiff);
}

void DebugMonitor::printResource() {
    if (_resourceEnabled) {
        // Heap Monitor Example
/*
        mbed_stats_heap_t heap_stats;

        printf("Starting heap stats example\r\n");

        void *allocation = malloc(1000);
        printf("Freeing 1000 bytes\r\n");

        mbed_stats_heap_get(&heap_stats);
        printf("Current heap: %lu\r\n", heap_stats.current_size);
        printf("Max heap size: %lu\r\n", heap_stats.max_size);

        free(allocation);

        mbed_stats_heap_get(&heap_stats);
        printf("Current heap after: %lu\r\n", heap_stats.current_size);
        printf("Max heap size after: %lu\r\n", heap_stats.max_size);
*/

        // Stack Monitor Example
/*        printf("Starting stack stats example\r\n");

        osThreadId main_id = osThreadGetId();

        osEvent info;
        info = osThreadGet
        info = _osThreadGetInfo(main_id, osThreadInfoStackSize);
        if (info.status != osOK) {
            error("Could not get stack size");
        }
        uint32_t stack_size = (uint32_t)info.value.v;
        info = _osThreadGetInfo(main_id, osThreadInfoStackMax);
        if (info.status != osOK) {
            error("Could not get max stack");
        }
        uint32_t max_stack = (uint32_t)info.value.v;

        printf("Stack used %li of %li bytes\r\n", max_stack, stack_size);
*/
    }
}
