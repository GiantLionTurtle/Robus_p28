
#include "UnitTests.hpp"

#include "Utils/Vec.hpp"

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

} // !Tests

} // !p28
