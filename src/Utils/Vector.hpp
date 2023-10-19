
#ifndef P28_VECTOR_HPP_
#define P28_VECTOR_HPP_

namespace p28 {

// std::vector, but bad
template<typename T>
struct Vector {
private:
	T* mData;
	unsigned int mSize { 0 };
	unsigned int mCapacity { 0 };

public:
	Vector() = default;
	Vector(unsigned int size_)
		: mSize(size_)
	{
		reallocate(size_);
	}
	Vector(unsigned int size_, T value)
		: mSize(size_)
	{
		reallocate(size);

		for(unsigned int i = 0; i < size(); ++i) {
			at(i) = value;
		}
	}
	~Vector()
	{
		delete mData;
	}

	size_t size() const { return mSize; }

	void push_back(T value)
	{
		if(mCapacity < mSize+1) {
			reallocate(mCapacity + 4); // We are on an arduino, cannot afford to allocate capacity * 2
		}
	}

	T& operator[](unsigned int index)
	{
		return at(index);
	}
	T const& operator[](unsigned int index) const
	{
		return at(index);
	}

	T& at(unsigned int index)
	{
#ifdef DEBUG_MODE
		if(index >= size()) {
			Serial.print("Index out of bound ");
			Serial.print(index);
			Serial.print(" vs ");
			Serial.println(size());
		}
#endif
		return mData[index];
	}
	T const& at(unsigned int index) const
	{
#ifdef DEBUG_MODE
		if(index >= size()) {
			Serial.print("Index out of bound ");
			Serial.print(index);
			Serial.print(" vs ");
			Serial.println(size());
		}
#endif
		return mData[index];
	}

private:
	void reallocate(unsigned int capacity)
	{
		T* new_data = new T[capacity];

		for(int i = 0; i < size(); ++i) {
			new_data = mData[i];
		}
		delete[] mData;
		mData = new_data;
	}
};

} // !p28

#endif