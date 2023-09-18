
#ifndef P28_HARDWARETEST_HPP_
#define P28_HARDWARETEST_HPP_

/*
	Hardware tests are used to check if every
	actuators are functionnal and do not provide
	extensive safety checks or control systems
*/

namespace p28 {

void test_lights();

void test_motors_and_encoders();

/*
	@function test_all

	Squence: 
	
		# Test lights
		Green Leds flash 4 times
		1000ms delay
		Red Leds flash 4 times
		1000ms delay

		# Test motors/Encoders
		Turn on itself right for 1000ms
		Flashes red/green 4 times for failure/success

		Turn on itself left for 1000ms
		Flashes red/green 4 times for failure/success

		Goes forward for 5000ms
		Flashes red/green 4 times for failure/success

		Goes backward for 5000ms
		Flashes red/green 4 times for failure/success
*/
void test_all();

} // !p28

#endif