
#ifndef P28_VEC_HPP_
#define P28_VEC_HPP_

#include <Arduino.h>

/*
	Minimal vector utilities
*/

namespace p28 {
namespace mt {

template<typename T>
struct Vec2_any {

	// Vec2 are used for
	// 1. Maths (x, y)
	// 2. Storing information about the drivebase (left, right)
	// 3. Storing a value->target pair
	union { T x; T left; T value; };
	union { T y; T right; T target; };

	Vec2_any() = default;
	Vec2_any(T v)
		: x(v)
		, y(v)
	{

	}
	Vec2_any(T x_, T y_)
		: x(x_)
		, y(y_)
	{

	}
	T& operator[](uint8_t ind)
	{
		// Hopefuly the switch 
		// is mapped at compile time 
		switch(ind) {
		case 0:
			return x;
		case 1:
			return y;
		}
		return x; // Not great
	}
	T operator[](uint8_t ind) const
	{
				// Hopefuly the switch 
		// is mapped at compile time 
		switch(ind) {
		case 0:
			return x;
		case 1:
			return y;
		}
		return T();
	}

	inline Vec2_any<T>& operator+=(Vec2_any<T> const& other)
	{
		x += other.x;
		y += other.y;
		return *this;
	}
	inline Vec2_any<T>& operator+=(T scalar)
	{
		x += scalar;
		y += scalar;
		return *this;
	}
	inline Vec2_any<T>& operator-=(Vec2_any<T> const& other)
	{
		x -= other.x;
		y -= other.y;
		return *this;
	}
	inline Vec2_any<T>& operator-=(T scalar)
	{
		x -= scalar;
		y -= scalar;
		return *this;
	}
	inline Vec2_any<T>& operator*=(T scalar)
	{
		x *= scalar;
		y *= scalar;
		return *this;
	}
	inline Vec2_any<T>& operator/=(T scalar)
	{
		x /= scalar;
		y /= scalar;
		return *this;
	}
};

using Vec2 = Vec2_any<float>;
using i32Vec2 = Vec2_any<int32_t>;
using boolVec2 = Vec2_any<bool>;
template<typename T>
using ValTarg = Vec2_any<T>;

// ----- Operations with other vectors ----- //
template<typename T>
inline Vec2_any<T> operator+(Vec2_any<T> const& lhs, Vec2_any<T> const& rhs)
{
	return { lhs.x + rhs.x, lhs.y + rhs.y };
}
template<typename T>
inline Vec2_any<T> operator-(Vec2_any<T> const& lhs, Vec2_any<T> const& rhs)
{
	return { lhs.x - rhs.x, lhs.y - rhs.y };
}
template<typename T>
inline T dot(Vec2_any<T> const& lhs, Vec2_any<T> const& rhs)
{
	return lhs.x * rhs.x + lhs.y * rhs.y;
}

// Cross product is technically not defined in 2d
// sue me
template<typename T>
inline T cross(Vec2_any<T> const& lhs, Vec2_any<T> const& rhs)
{
	return lhs.x * rhs.y - lhs.y * rhs.x;
}

// ----- Operations with scalars ----- //
template<typename T>
inline Vec2_any<T> operator+(Vec2_any<T> const& lhs, T const& rhs)
{
	return { lhs.x + rhs, lhs.y + rhs };
}
template<typename T>
inline Vec2_any<T> operator-(Vec2_any<T> const& lhs, T const& rhs)
{
	return { lhs.x - rhs, lhs.y - rhs };
}
template<typename T>
inline Vec2_any<T> operator*(Vec2_any<T> const& lhs, T const& rhs)
{
	return { lhs.x * rhs, lhs.y * rhs };
}
template<typename T>
inline Vec2_any<T> operator/(Vec2_any<T> const& lhs, T const& rhs)
{
	return { lhs.x / rhs, lhs.y / rhs };
}

template<typename T>
inline Vec2_any<T> operator+(T const& lhs, Vec2_any<T> const& rhs)
{
	return { lhs + rhs.x, lhs + rhs.y };
}
template<typename T>
inline Vec2_any<T> operator-(T const& lhs, Vec2_any<T> const& rhs)
{
	return { lhs - rhs.x, lhs - rhs.y };
}
template<typename T>
inline Vec2_any<T> operator*(T const& lhs, Vec2_any<T> const& rhs)
{
	return { lhs * rhs.x, lhs * rhs.y };
}
template<typename T>
inline Vec2_any<T> operator/(T const& lhs, Vec2_any<T> const& rhs)
{
	return { lhs / rhs.x, lhs / rhs.y };
}

// ----- Measurements ----- //

template<typename T>
inline T magnitude2(Vec2_any<T> const& vec)
{
	return vec.x * vec.x + vec.y * vec.y;
}
template<typename T>
inline T magnitude(Vec2_any<T> const& vec)
{
	return sqrt(magnitude2(vec));
}

template<typename T>
inline T distance2(Vec2_any<T> const& first, Vec2_any<T> const& second)
{
	float x_ = second.x-first.x;
	float y_ = second.y-first.y;
	return x_*x_ + y_*y_;
}
template<typename T>
inline T distance(Vec2_any<T> const& first, Vec2_any<T> const& second)
{
	return sqrt(distance2(first, second));
}

// Smallest angle between two vectors in radians, [-pi, pi], < 0 means clockwise
template<typename T>
inline T signed_angle(Vec2_any<T> const& lhs, Vec2_any<T> const& rhs)
{
	// https://stackoverflow.com/questions/5188561/signed-angle-between-two-3d-vectors-with-same-origin-within-the-same-plane
	return atan2(cross(lhs, rhs), dot(lhs, rhs));
}
// Counter clockwise angle between two vectors
template<typename T>
inline T angle(Vec2_any<T> const& lhs, Vec2_any<T> const& rhs)
{
	// https://math.stackexchange.com/questions/2718543/how-to-calculate-the-angle-between-2-vectors-in-a-plane
	Vec2_any<T> perp_lhs(-lhs.y, lhs.x);
	return atan2(dot(rhs, perp_lhs), dot(lhs, rhs));
}

// ----- Comparisons & access ----- //
template<typename T>
inline bool operator==(Vec2_any<T> const& lhs, Vec2_any<T>  const& rhs)
{
	return lhs.x == rhs.x && lhs.y == rhs.y;
}

template<typename T>
inline bool epsilon_equal(T const& lhs, T const& rhs, T epsilon)
{
	return lhs+epsilon >= rhs && lhs-epsilon <= rhs;
}
template<typename T>
inline bool epsilon_equal(Vec2_any<T> const& lhs, Vec2_any<T>  const& rhs, T epsilon)
{
	return epsilon_equal(distance(lhs, rhs), 0.0f, epsilon);
}
template<typename T>
inline bool epsilon_equal2(Vec2_any<T> const& lhs, Vec2_any<T>  const& rhs, T epsilon2)
{
	return epsilon_equal(distance2(lhs, rhs), 0.0f, epsilon2);
}

// ----- Transformations ----- //
template<typename T>
inline Vec2_any<T> operator-(Vec2_any<T> const& vec)
{
	return mt::Vec2{-vec.x, -vec.y};
}

template<typename T>
inline Vec2_any<T> rotate(Vec2_any<T> const& vec, T const& angle_rad)
{
	// https://en.wikipedia.org/wiki/Rotation_matrix#In_two_dimensions

	// Cos & sin are expensive, store them in temps
	T cos_ = cos(angle_rad);
	T sin_ = sin(angle_rad);

	return { vec.x * cos_ - vec.y * sin_, vec.x * sin_ + vec.y * cos_ };
}
template<typename T>
inline Vec2_any<T> ccw_perpendicular(Vec2_any<T> const& vec)
{
	return mt::Vec2{ -vec.y, vec.x };
}
template<typename T>
inline Vec2_any<T> cw_perpendicular(Vec2_any<T> const& vec)
{
	return -ccw_perpendicular(vec);
}

template<typename T>
inline Vec2_any<T> normalize(Vec2_any<T> const& vec)
{
	T mag = magnitude(vec);
	return { vec.x / mag, vec.y / mag };
}

// Print
template<typename T>
inline void print(Vec2_any<T> const& vec, int decimals=2)
{
	Serial.print("[");
	Serial.print(vec.x);
	Serial.print(",  ");
	Serial.print(vec.y);
	Serial.print("]");
}
template<>
inline void print(Vec2 const& vec, int decimals)
{
	Serial.print("[");
	Serial.print(vec.x, decimals);
	Serial.print(",  ");
	Serial.print(vec.y, decimals);
	Serial.print("]");
}

} // !mt
} // !p28


#endif