#ifndef __FIXED_T__H
#define __FIXED_T__H

#include <stdint.h>
#include <math.h>

typedef int32_t fx_t;
typedef int64_t fx_ext_t;

class __attribute__((packed)) fixed {
private:
	static const uint8_t FIXED_BITS;
	static const uint8_t FIXED_FBITS;
	static const fx_t FIXED_ONE;
	static const fx_ext_t FIXED_K;
	static const double FIXED_ONE_D;
	static const double FIXED_ONE_D_RECP;

	// These makes it thread-unsafe
	static fx_t temp;
	static int loop;
	static fx_ext_t temp_ext;

public:
	fx_t value;

	// The actual handling of fixed point calculations
	static inline fx_t fixed_from_int(int a) {
		return fx_t(a << FIXED_FBITS);
	}

	static inline int fixed_to_int(fx_t a) {
		return int(a >> FIXED_FBITS);
	}

	static inline fx_t fixed_from_double(double a) {
		return fx_t(a * FIXED_ONE_D);
	}

	static inline double fixed_to_double(fx_t a) {
		return double(a) * FIXED_ONE_D_RECP;
	}

	static fx_t fixed_multiply(fx_t a, fx_t b) {
		temp_ext = fx_ext_t(a) * fx_ext_t(b);
		temp_ext += FIXED_K; // Rounding; mid values are rounded up
		return fx_t(temp_ext >> FIXED_FBITS); // Correct by dividing by base
	}

	static fx_t fixed_divide(fx_t a, fx_t b) {
		// pre-multiply by the base
		temp_ext = fx_ext_t(fx_ext_t(a) << FIXED_FBITS);
		// So the result will be rounded ; mid values are rounded up.
		temp_ext += b / 2;
		return fx_t(temp_ext / b);
	}

	fixed() :
			value(0) {
	}

	// additional bool argument is needed to separate from constructor from int
	// the truth value of the bool doesn't matter
	fixed(const fx_t b, bool is_fixed_point) :
			value(b) {
	}

	fixed(const fixed& b) :
			value(b.value) {
	}

	fixed(const int b) :
			value(fixed_from_int(b)) {
	}

	fixed(const double b) :
			value(fixed_from_double(b)) {
	}

	operator int() {
		return fixed_to_int(value);
	}

	operator double() {
		return fixed_to_double(value);
	}

	fixed operator-() {
		return fixed(-value, true);
	}

	fixed& operator=(const fixed& b) {
		value = b.value;
		return *this;
	}

	fixed& operator=(const int b) {
		value = fixed_from_int(b);
		return *this;
	}

	fixed& operator=(const double b) {
		value = fixed_from_double(b);
		return *this;
	}

	fixed& operator+=(const fixed& b) {
		value = value + b.value;
		return *this;
	}

	fixed& operator+=(const int b) {
		value = value + fixed_from_int(b);
		return *this;
	}

	fixed& operator+=(const double b) {
		value = value + fixed_from_double(b);
		return *this;
	}

	fixed& operator-=(const fixed& b) {
		value = value - b.value;
		return *this;
	}

	fixed& operator-=(const int b) {
		value = value - fixed_from_int(b);
		return *this;
	}

	fixed& operator-=(const double b) {
		value = value - fixed_from_double(b);
		return *this;
	}

	fixed& operator*=(const fixed& b) {
		value = fixed_multiply(value, b.value);
		return *this;
	}

	fixed& operator*=(const int b) {
		value = fixed_multiply(value, fixed_from_int(b));
		return *this;
	}

	fixed& operator*=(const double b) {
		value = fixed_multiply(value, fixed_from_double(b));
		return *this;
	}

	fixed& operator/=(const fixed& b) {
		value = fixed_divide(value, b.value);
		return *this;
	}

	fixed& operator/=(const int b) {
		value = fixed_divide(value, fixed_from_int(b));
		return *this;
	}

	fixed& operator/=(const double b) {
		value = fixed_divide(value, fixed_from_double(b));
		return *this;
	}

	fixed operator+(const fixed& b) {
		return fixed(value + b.value, true);
	}

	fixed operator+(const int b) {
		return fixed(value + fixed_from_int(b), true);
	}

	fixed operator+(const double b) {
		return fixed(value + fixed_from_double(b), true);
	}

	fixed operator-(const fixed& b) {
		return fixed(value - b.value, true);
	}

	fixed operator-(const int b) {
		return fixed(value - fixed_from_int(b), true);
	}

	fixed operator-(const double b) {
		return fixed(value - fixed_from_double(b), true);
	}

	fixed operator*(const fixed& b) {
		return fixed(fixed_multiply(value, b.value), true);
	}

	fixed operator*(const int b) {
		return fixed(fixed_multiply(value, fixed_from_int(b)), true);
	}

	fixed operator*(const double b) {
		return fixed(fixed_multiply(value, fixed_from_double(b)), true);
	}

	fixed operator/(const fixed& b) {
		return fixed(fixed_divide(value, b.value), true);
	}

	fixed operator/(const int b) {
		return fixed(fixed_divide(value, fixed_from_int(b)), true);
	}

	fixed operator/(const double b) {
		return fixed(fixed_divide(value, fixed_from_double(b)), true);
	}

	bool operator >(const fixed& b) {
		return value > b.value;
	}

	fixed& operator++() {
		value += FIXED_ONE;
		return *this;
	}

	fixed& operator--() {
		value -= FIXED_ONE;
		return *this;
	}

	fixed operator++(int) {
		fixed res(value, true);
		value += FIXED_ONE;
		return res;
	}

	fixed operator--(int) {
		fixed res(value, true);
		value -= FIXED_ONE;
		return res;
	}

	bool operator >=(const fixed& b) {
		return value >= b.value;
	}

	bool operator <(const fixed& b) {
		return value < b.value;
	}

	bool operator <=(const fixed& b) {
		return value <= b.value;
	}

	bool operator ==(const fixed& b) {
		return value == b.value;
	}

	bool operator !=(const fixed& b) {
		return value != b.value;
	}

	bool operator >(const int b) {
		return value > fixed_from_int(b);
	}

	bool operator >=(const int b) {
		return value >= fixed_from_int(b);
	}

	bool operator <(const int b) {
		return value < fixed_from_int(b);
	}

	bool operator <=(const int b) {
		return value <= fixed_from_int(b);
	}

	bool operator ==(const int b) {
		return value == fixed_from_int(b);
	}

	bool operator !=(const int b) {
		return value != fixed_from_int(b);
	}

	fixed abs() {
		return fixed(value >= 0 ? value : -value, true);
	}
};

fixed operator+(const int a, const fixed& b);
fixed operator+(const double a, const fixed& b);
fixed operator-(const int a, const fixed& b);
fixed operator-(const double a, const fixed& b);
fixed operator*(const int a, const fixed& b);
fixed operator*(const double a, const fixed& b);
fixed operator/(const int a, const fixed& b);
fixed operator/(const double a, const fixed& b);

#endif // __FIXED_T__H

