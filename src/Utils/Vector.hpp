
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
	Vector(Vector<T>&& other)
	{
		mData = other.mData;
		other.mData = NULL;
		mSize = other.mSize;
		mCapacity = other.mCapacity;
	}
	Vector& operator=(Vector<T>&& other)
	{
		mData = other.mData;
		other.mData = NULL;
		mSize = other.mSize;
		mCapacity = other.mCapacity;
		return *this;
	}
	Vector(Vector<T> const& other)
	{
		deep_copy(other.mData, other.mSize);
	}
	Vector& operator=(Vector<T> const& other)
	{
		deep_copy(other.mData, other.mSize);
		return *this;
	}

	~Vector()
	{
		delete mData;
	}

	unsigned int size() const { return mSize; }
	unsigned int capacity() const { return mCapacity; }

	void push_back(T value)
	{
		if(mCapacity < mSize+1) {
			reallocate(mCapacity + 4); // We are on an arduino, cannot afford to allocate capacity * 2
		}
		mData[mSize] = value;
		mSize++;
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

		for(unsigned int i = 0; i < size(); ++i) {
			new_data[i] = mData[i];
		}
		delete[] mData;
		mData = new_data;
		mCapacity = capacity;
	}
	void deep_copy(T* ext_data, unsigned int size_)
	{
		if(size_ > mCapacity) {
			reallocate(size_+4);
		}
		for(unsigned int i = 0; i < size_; ++i) {
			mData[i] = ext_data[i];
		}
		mSize = size_;
	}
};

} // !p28

#endif