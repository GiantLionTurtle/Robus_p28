
#ifndef P28_PAIR_HPP_
#define P28_PAIR_HPP_

/*
	Crappy look alike to std::pair and std::tie
*/

namespace p28 {

template<typename T1_, typename T2_>
struct Pair {
	T1_ first;
	T2_ second;

	Pair(T1_ first_, T2_ second_)
		: first(first_)
		, second(second_)
	{

	}

	template<typename ext_T1_, typename ext_T2_>
	Pair& operator=(Pair<ext_T1_, ext_T2_> const& other)
	{
		first = other.first;
		second = other.second;
		return *this;
	}
};

template<typename T1_, typename T2_>
Pair<T1_&, T2_&> tie(T1_& first, T2_& second)
{
	return Pair<T1_&, T2_&>(first, second);
}

} // !p28

#endif