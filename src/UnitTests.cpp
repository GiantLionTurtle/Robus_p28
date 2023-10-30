
#include "UnitTests.hpp"

#include "Utils/Vec2.hpp"
#include "Drivebase.hpp"

namespace p28 {

namespace Tests {

// Affiche le nombre de succès et les statistiques de la
// période de test
void print_successRate(float successes, float tries)
{
	float success_rate = successes/tries * 100;
	Serial.print("Passed ");
	Serial.print(static_cast<int>(successes));
	Serial.print(" out of ");
	Serial.print(static_cast<int>(tries));
	Serial.print(" (");
	Serial.print(success_rate);
	Serial.println("%)");
}


void vector_maths()
{
	struct Trial {
		mt::Vec2 vec_a;
		mt::Vec2 vec_b;
		float expected;
	};

	const int n_trials = 10;
	Trial trials[n_trials] {
		Trial { .vec_a=mt::Vec2(0.0, 1.0), .vec_b=mt::Vec2(1.0, 0.0), .expected=-PI/2 },
		Trial { .vec_a=mt::Vec2(1.0, 0.0), .vec_b=mt::Vec2(0.0, 1.0), .expected=PI/2 },

		Trial { .vec_a=mt::Vec2(1.0, 5.0), .vec_b=mt::Vec2(-5.0, 2.0), .expected=1.3877 },

		Trial { .vec_a=mt::Vec2(2.0, 2.0), .vec_b=mt::Vec2(-2.0, -2.0), .expected=PI },
		Trial { .vec_a=mt::Vec2(2.0, 2.0), .vec_b=mt::Vec2(-2.0, -1.0), .expected=2.8198 },
		Trial { .vec_a=mt::Vec2(2.0, 2.0), .vec_b=mt::Vec2(-2.0, -3.0), .expected=-2.9442 },

		Trial { .vec_a=mt::Vec2(1.0, 1.0), .vec_b=mt::Vec2(-2.0, 2.0), .expected=PI/2 },
		Trial { .vec_a=mt::Vec2(-2.0, 2.0), .vec_b=mt::Vec2(4.0, 4.0), .expected=-PI/2 },

		Trial { .vec_a=mt::Vec2(-2.0, -2.0), .vec_b=mt::Vec2(3.0, -3.0), .expected=PI/2 },
		Trial { .vec_a=mt::Vec2(5.0, -5.0), .vec_b=mt::Vec2(-2.0, -2.0), .expected=-PI/2 },
	};

	Serial.println("Testing vector maths");
	// https://onlinemschool.com/math/assistance/vector/angl/
	int successes = 0;
	for(int i = 0; i < n_trials; ++i) {
		float out = mt::signed_angle(trials[i].vec_a, trials[i].vec_b);
		if(mt::epsilon_equal(out, trials[i].expected, 0.0001f)) {
			successes++;
		} else {
			Serial.print("Failed ind=");
			Serial.println(i);
		}
	}
	print_successRate(successes, n_trials);
}

bool epsilon_equal(DrivebaseState st1, DrivebaseState st2, float epsilon)
{
	return 	mt::epsilon_equal(st1.pos, st2.pos, epsilon) && 
			mt::epsilon_equal(st1.heading, st2.heading, epsilon);
}
void forward_kinematics()
{
	struct Trial {
		DrivebaseState init;
		mt::i32Vec2 encTicks;
		float delta_s;
		DrivebaseState expected;
	};

	const int n_trials = 1;
	Trial trials[n_trials] {
		Trial { .init=DrivebaseState{}, .encTicks=dist_to_ticks(mt::Vec2(1.0)), .delta_s=2.0, .expected=DrivebaseState(mt::Vec2(0.0, 1.0))}
	};

	Serial.println("Testing forward kinematics");
	// https://onlinemschool.com/math/assistance/vector/angl/
	int successes = 0;
	for(int i = 0; i < n_trials; ++i) {
		DrivebaseState out = trials[i].init.update_kinematics(mt::i32Vec2(0), trials[i].encTicks, trials[i].delta_s);
		if(epsilon_equal(out, trials[i].expected, 0.0001f)) {
			successes++;
		} else {
			Serial.print("Failed ind=");
			Serial.println(i);
		}
	}
	print_successRate(successes, n_trials);
}

void print_helper_accelProfile(float time, float current_vel, float traveled_dist)
{
	Serial.print(time);
	Serial.print(",  ");
	Serial.print(current_vel);
	Serial.print(",  ");
	Serial.println(traveled_dist, 8);

}
void acceleration_profile()
{
	// Use the output of this test in a spreadsheet to check the acceleration profile

	struct Trial {
		float init_vel;
		float end_vel;
		float dist_to_travel;
		float accel;
		float time_step;
	};

	const int n_trials = 4;
	Trial trials[n_trials] {
		Trial{ .init_vel=0.0, .end_vel=0.0, .dist_to_travel=5.0, .accel=0.5, .time_step=0.05 },
		Trial{ .init_vel=0.0, .end_vel=0.0, .dist_to_travel=5.0, .accel=1.3, .time_step=0.05 },
		Trial { .init_vel=1.2, .end_vel=1.9, .dist_to_travel=3.0, .accel=1.3, .time_step=0.05 },
		Trial { .init_vel=1.9, .end_vel=0.8, .dist_to_travel=2.5, .accel=1.2, .time_step=0.05 }
	};

	for(int i = 0; i < n_trials; ++i) {
		Serial.print(" --- Start test ");
		Serial.print(i);
		Serial.println(" ---");
		float traveled_dist = 0;
		float current_vel = trials[i].init_vel;
		float time = 0;
		while(traveled_dist < trials[i].dist_to_travel && !mt::epsilon_equal(traveled_dist, trials[i].dist_to_travel, 0.001f)) {
			traveled_dist += current_vel * trials[i].time_step;
			time += trials[i].time_step;

			current_vel = velocity_for_point(current_vel, trials[i].end_vel, trials[i].dist_to_travel-traveled_dist, trials[i].accel, trials[i].time_step);
			print_helper_accelProfile(time, current_vel, traveled_dist);
		}
	}
}

bool arcs_equal(Arc a1, Arc a2, float epsilon)
{
	return 	mt::epsilon_equal(a1.end, a2.end, epsilon) && 
			mt::epsilon_equal(a1.tengeantStart, a2.tengeantStart, epsilon) &&
			mt::epsilon_equal(a1.radius, a2.radius, epsilon) &&
			 mt::epsilon_equal(a1.length, a2.length, epsilon);
}

void arc_generation()
{
	struct Trial {
		mt::Vec2 start, end, end_heading;
		Arc expected;
	};

	const int n_trials = 4;
	Trial trials[n_trials] = {
		Trial { .start=mt::Vec2(0.0, 0.0), .end=mt::Vec2(0.0, 1.0), .end_heading=mt::Vec2(1.0, 0.0), 
					.expected=Arc{.tengeantStart=mt::Vec2(-1.0, 0.0), .end=mt::Vec2(0.0, 1.0), .radius=-0.5, .length=0.5*PI } },
		Trial { .start=mt::Vec2(0.0, 0.0), .end=mt::Vec2(0.0, 1.0), .end_heading=mt::Vec2(-1.0, 0.0), 
					.expected=Arc{.tengeantStart=mt::Vec2(1.0, 0.0), .end=mt::Vec2(0.0, 1.0), .radius=0.5, .length=0.5*PI } },

		// Straigth line
		Trial { .start=mt::Vec2(2.0, 2.0), .end=mt::Vec2(4, 4), .end_heading=mt::normalize(mt::Vec2(1, 1)), 
					.expected=Arc{.tengeantStart=mt::normalize(mt::Vec2(1, 1)), .end=mt::Vec2(4.0, 4.0), .radius=kInfinity, .length=sqrt(8) } },
		Trial { .start=mt::Vec2(0.0, 0.0), .end=mt::Vec2(1.0, 1.0), .end_heading=mt::Vec2(1.0, 0.0),
					.expected=Arc{.tengeantStart=mt::Vec2(0.0, 1.0), .end=mt::Vec2(1.0, 1.0), .radius=-1.0, .length=PI/2}}
	};

	Serial.println(" --- Testing arc generation --- ");
	int successes = 0;
	for(int i = 0; i < n_trials; ++i) {
		Arc out = arc_from_targetHeading(trials[i].start, trials[i].end, trials[i].end_heading);
		if(arcs_equal(out, trials[i].expected, 0.001)) {
			successes++;
		} else {
			out.print();

			Serial.print(" --- Failed test ");
			Serial.print(i);
			Serial.println(" --- ");
		}
	}
	print_successRate(successes, n_trials);
}

void vectors()
{
	const int n_tests = 8;
	int successes = 0;

	Serial.println(" --- Vector tests --- ");
	Serial.println("[assignment]");

	Vector<int> vector;
	for(int i = 0; i < 3; ++i) {
		vector.push_back(i);
	}

	if(vector.size() == 3) {
		successes++;

		for(int i = 0; i < 3; ++i) {
			if(vector[i] == i)
				successes++;
		}
	}

	Serial.println("[copy]");

	Vector<int> copy = vector;
	if(copy.size() == 3) {
		successes++;

		for(int i = 0; i < 3; ++i) {
			if(copy[i] == i)
				successes++;
		}	
	}

	print_successRate(successes, n_tests);
}

void near_equality()
{
	struct Trial {
		mt::Vec2 a;
		mt::Vec2 b;
		float epsilon2;
		bool expected;
	};

	const int n_trials = 1;
	Trial trials[n_trials] = {
		Trial {.a=mt::Vec2(0.0, 0.0), .b=mt::Vec2(0.0, 0.5), .epsilon2=kPathFollower_epsilon2, .expected=false }
	};

	Serial.println(" --- epsilon equal test --- ");

	int successes = 0;
	for(int i = 0; i < n_trials; ++i) {
		if(mt::epsilon_equal2(trials[i].a, trials[i].b, trials[i].epsilon2) == trials[i].expected) {
			successes++;
		} else {
			Serial.print(" --- Failed test ");
			Serial.print(i);
			Serial.println(" --- ");
		}
	}

	print_successRate(successes, n_trials);
}

} // !Tests

} // !p28
