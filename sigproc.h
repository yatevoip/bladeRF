/**
 * sigproc.h
 * This file is part of the Yate-BTS Project http://www.yatebts.com
 *
 * Transceiver Signal Process Library related classes
 *
 * Yet Another Telephony Engine - Base Transceiver Station
 * Copyright (C) 2014 Null Team Impex SRL
 *
 * This software is distributed under multiple licenses;
 * see the COPYING file in the main directory for licensing
 * information for this specific distribution.
 *
 * This use of this software may be subject to additional restrictions.
 * See the LEGAL file in the main directory for details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SIGPROC_H
#define SIGPROC_H

#include <yateclass.h>
#include <math.h>

namespace TelEngine {

class SigProcUtils;                      // Utility functions
class Complex;                           // A Complex (float) number
template <class Obj> class SigProcVector;// Basic vector template
class ComplexArray;                      // A Complex array

#define GSM_SYMBOL_RATE 13e6/48 // 13 * 10^6 / 48
#define BITS_PER_TIMESLOT 156.25
#define PI M_PI
//#define DUMP_COMPLEX

/**
 * Processing library utility functions, not related to signal processing
 * @short Processing library utilities
 */
class SigProcUtils
{
public:
    /**
     * Copy memory
     * @param dest Destination buffer
     * @param len Destination buffer length
     * @param src Pointer to data to copy
     * @param n The number of bytes to copy
     * @param objLen Object (element) size in bytes
     * @param offs Optional offset in destination
     * @return The number of elements copied
     */
    static unsigned int copy(void* dest, unsigned int len,
	const void* src, unsigned int n,
	unsigned int objSize = 1, unsigned int offs = 0);
};


/**
 * This class implements a complex number
 * @short A Complex (float) number
 */
class Complex
{
public:
    /**
     * Constructor
     * Default (0,0i)
     */
    inline Complex()
	: m_real(0), m_imag(0)
	{}

    /**
     * Constructor
     * @param real The real part of the complex number
     * @param imag The imaginary part of a complex number
     */
    inline Complex(float real, float imag = 0)
	: m_real(real), m_imag(imag)
	{}

    /**
     * Copy Constructor
     * @param c The source complex number
     */
    inline Complex(const Complex& c)
	: m_real(c.real()), m_imag(c.imag())
	{}

    /**
     * Obtain the real part of the complex number
     * @return The real part
     */
    inline float real() const
	{ return m_real; }

    /**
     * Set the real part of the complex number
     * @param r The new real part value
     */
    inline void real(float r)
	{ m_real = r; }

    /**
     * Obtain the imaginary part of a complex number
     * @return The imaginary part
     */
    inline float imag() const
	{ return m_imag; }

    /**
     * Set the imaginary part of the complex number
     * @param i The new imaginary part value
     */
    inline void imag(float i)
	{ m_imag = i; }

    /**
     * Set data
     * @param r The real part of the complex number
     * @param i The imaginary part of a complex number
     */
    inline void set(float r = 0, float i = 0) {
	    real(r);
	    imag(i);
	}

    inline Complex& operator=(float real) {
	    set(real);
	    return *this; 
	}

    inline Complex& operator=(const Complex& c) {
	    set(c.real(),c.imag());
	    return *this; 
	}

    inline Complex& operator+=(float real) { 
	    m_real += real;
	    return *this;
	}

    inline Complex& operator+=(const Complex& c)
	{ return sum(*this,*this,c); }

    inline Complex& operator-=(float real) {
	    m_real -= real;
	    return *this; 
	}

    inline Complex& operator-=(const Complex& c)
	{ return diff(*this,*this,c); }

    inline Complex operator*(const Complex& c) const {
	    Complex tmp;
	    return multiply(tmp,*this,c);
	}

    inline Complex operator+(const Complex& c) const { 
	    Complex tmp;
	    return sum(tmp,*this,c);
	}

    /**
     * Obtain tha absolute value of a complex number
     * @return The absolute value.
     */
    inline float abs() const
	{ return ::sqrt(m_real*m_real + m_imag*m_imag);}

    /**
     * Multiply this number with its conjugate
     * @return The result
     */
    inline float mulConj() const
	{ return m_real * m_real + m_imag * m_imag; }

    void dump(String& dest) const;

    static inline Complex& exp(Complex& dest, float r, float i) {
	    dest.set(::expf(r) * ::cosf(i),::expf(r) * ::sinf(i));
	    return dest;
	}

    static inline Complex& multiply(Complex& dest, const Complex& c1,
	const Complex& c2) {
	    dest.set(c1.real() * c2.real() - c1.imag() * c2.imag(),
		c1.real() * c2.imag() + c1.imag() * c2.real());
	    return dest;
	}

    static inline Complex& multiplyF(Complex& dest, const Complex& c, float real) {
	    dest.set(c.real() * real,c.imag() * real);
	    return dest;
	}

    /**
     * Multiply two complex arrays
     * Multiply c1 and c2 and put the result in dest.
     * The number of elements to process is min(len,len1,len2)
     * @param dest Destination array
     * @param len Destination array length
     * @param c1 Pointer to first array
     * @param len1 First array length
     * @param c2 Second array
     * @param len2 Second array length
     */
    static void multiply(Complex* dest, unsigned int len,
	const Complex* c1, unsigned int len1, const Complex* c2, unsigned int len2);

    /**
     * Multiply two complex arrays.
     * Multiply c1 and c2 and put the result in c1.
     * The number of elements to process is min(len1,len2)
     * @param c1 Pointer to first array
     * @param len1 First array length
     * @param c2 Second array
     * @param len2 Second array length
     */
    static inline void multiply(Complex* c1, unsigned int len1,
	const Complex* c2, unsigned int len2)
	{ multiply(c1,len1,c1,len1,c2,len2); }

    /**
     * Compute the sum of two complex numbers.
     * Sum c1 and c2 and put the result in dest
     * @param dest Destination number
     * @param c1 First number
     * @param c2 Second number
     */
    static inline Complex& sum(Complex& dest, const Complex& c1, const Complex& c2) {
	    dest.set(c1.real() + c2.real(),c1.imag() + c2.imag());
	    return dest;
	}

    /**
     * Compute the sum of two complex arrays.
     * Sum c1 and c2 and put the result in dest.
     * The number of elements to process is min(len,len1,len2)
     * @param dest Destination array
     * @param len Destination array length
     * @param c1 Pointer to first array
     * @param len1 First array length
     * @param c2 Second array
     * @param len2 Second array length
     */
    static void sum(Complex* dest, unsigned int len,
	const Complex* c1, unsigned int len1, const Complex* c2, unsigned int len2);

    /**
     * Compute the sum of two complex arrays.
     * Sum c1 and c2 and put the result in c1.
     * The number of elements to process is min(len1,len2)
     * @param c1 Pointer to first array
     * @param len1 First array length
     * @param c2 Second array
     * @param len2 Second array length
     */
    static inline void sum(Complex* c1, unsigned int len1,
	const Complex* c2, unsigned int len2)
	{ sum(c1,len1,c1,len1,c2,len2); }

    /**
     * Compute the sum of product between complex number and its conjugate
     *  of all numbers in an array
     * @param data Array to process
     * @param len Array length
     * @return The result
     */
    static float sumMulConj(const Complex* data, unsigned int len);

    /**
     * Compute the sum of product between 2 complex numbers
     * E.g. dest += c1 * c2
     * @param dest Destination number
     * @param c1 First number
     * @param c2 Second number
     */
    static inline void sumMul(Complex& dest, const Complex& c1, const Complex& c2) {
	    dest.set(dest.real() + (c1.real() * c2.real() - c1.imag() * c2.imag()),
		dest.imag() + (c1.real() * c2.imag() + c1.imag() * c2.real()));
	}

    /**
     * Compute the sum of product between 2 complex arrays
     * E.g. dest = SUM(c1[i] * c2[i])
     * @param dest Destination number
     * @param c1 Pointer to first array
     * @param len1 First array length
     * @param c2 Second array
     * @param len2 Second array length
     */
    static void sumMul(Complex& dest, const Complex* c1, unsigned int len1,
	const Complex* c2, unsigned int len2);

    /**
     * Multiply c1 by f. Add the result to c
     * @param c Destination number
     * @param c1 Complex number to multiply
     * @param f Float number to multiply
     */
    static inline void sumMulF(Complex& c, const Complex& c1, float f)
	{ c.set(c.real() + c1.real() * f,c.imag() + c1.imag() * f); }

    /**
     * Compute the difference of two complex numbers
     * @param dest Destination number
     * @param c1 First number
     * @param c2 Second number
     */
    static inline Complex& diff(Complex& dest, const Complex& c1, const Complex& c2) {
	    dest.set(c1.real() - c2.real(),c1.imag() - c2.imag());
	    return dest;
	}

    /**
     * Compute the difference of two complex arrays.
     * The number of elements to process is min(len,len1,len2)
     * @param dest Destination array
     * @param len Destination array length
     * @param c1 Pointer to first array
     * @param len1 First array length
     * @param c2 Second array
     * @param len2 Second array length
     */
    static void diff(Complex* dest, unsigned int len,
	const Complex* c1, unsigned int len1, const Complex* c2, unsigned int len2);

    /**
     * Compute the difference of two complex arrays.
     * The number of elements to process is min(len1,len2)
     * @param c1 Pointer to first array
     * @param len1 First array length
     * @param c2 Second array
     * @param len2 Second array length
     */
    static inline void diff(Complex* c1, unsigned int len1,
	const Complex* c2, unsigned int len2)
	{ diff(c1,len1,c1,len1,c2,len2); }

    /**
     * Replace each element in an array with its conjugate
     * @param c Pointer to data
     * @param len Data length
     */
    static void conj(Complex* c, unsigned int len);

    /**
     * Set complex elements from 16 bit integers array (pairs of real/imaginary parts)
     * @param dest Destination buffer
     * @param len Destination buffer length
     * @param src The source buffer
     * @param n Source buffer length in 16 bit integer elements
     * @param offs Optional offset in destination to start copy
     */
    static void setInt16(Complex* dest, unsigned int len,
	const int16_t* src, unsigned int n, unsigned int offs = 0);

    /**
     * Dump a Complex vector
     * @param dest The destination string
     * @param c The vector to dump
     * @param len The number of elements to dump
     * @param sep Optional separator between values
     */
    static void dump(String& dest, const Complex* c, unsigned int len,
	const char* sep = " ");

private:
    float m_real;                        // The Real part of the complex number
    float m_imag;                        // The imaginary part of the complex numbers
};


/**
 * Basic vector template.
 * NOTE: This template won't work for objects holding pointers,
 *  it copies data using raw memory copy
 * @short Basic vector template
 */
template <class Obj> class SigProcVector
{
public:
    /**
     * Constructor
     */
    inline SigProcVector()
	: m_data(0), m_length(0)
	{}

    /**
     * Constructor
     * @param len Array length
     */
    inline SigProcVector(unsigned int len)
	: m_data(0), m_length(0)
	{ assign(len); }

    /**
     * Constructor from data
     * @param ptr Source buffer
     * @param n Elements to copy from source buffer
     * @param len Optional vector length, 0 to use source buffer length
     */
    inline SigProcVector(const Obj* ptr, unsigned int n, unsigned int len = 0)
	: m_data(0), m_length(0) {
	    assign(len ? len : n);
	    copy(ptr,n);
	}

    /**
     * Constructor from another array
     * @param other Array to copy
     */
    inline SigProcVector(const SigProcVector<Obj>& other)
	: m_data(0), m_length(0)
	{ *this = other; }

    /**
     * Destructor
     */
    virtual ~SigProcVector()
	{ clear(); }

    /**
     * Retrieve array data pointer
     * @return Array data pointer, 0 if empty (not allocated)
     */
    inline Obj* data()
	{ return m_data; }

    /**
     * Retrieve array data pointer
     * @return A const pointer to array data, 0 if empty (not allocated)
     */
    inline const Obj* data() const
	{ return m_data; }

    /**
     * Retrieve array length
     * @return Array length
     */
    inline unsigned int length() const
	{ return m_length; }

    /**
     * Clear the array
     */
    inline void clear(bool del = true) {
	    if (!m_data)
		return;
	    if (del)
		delete[] m_data;
	    m_data = 0;
	    m_length = 0;
	}

    /**
     * Resize (re-alloc or free) this array if required size is not the same
     *  as the current one
     * @param len Required array length
     */
    inline void resize(unsigned int len) {
	    if (len != length())
		assign(len);
	}

    /**
     * Allocate space for array
     * @param len Array length
     */
    void assign(unsigned int len) {
	    clear();
	    if (!len)
		return;
	    m_data = new Obj[len];
	    if (m_data)
		m_length = len;
	    else
		Debug("SigProcVector",DebugFail,
		    "Failed to allocate len=%u obj_size=%u [%p]",
		    len,(unsigned int)(sizeof(Obj)),this);
	}

    /**
     * Copy elements from another array
     * @param ptr Pointer to data to copy
     * @param n The number of bytes to copy
     * @param offs Optional offset in our array
     * @return The number of elements copied
     */
    virtual unsigned int copy(const Obj* ptr, unsigned int n, unsigned int offs = 0)
	{ return SigProcUtils::copy(m_data,m_length,ptr,n,sizeof(Obj),offs); }

    /**
     * Take another array's data
     * @param other Array to steal data from
     */
    inline void steal(SigProcVector<Obj>& other) {
	    clear();
	    m_data = other.data();
	    m_length = other.length();
	    other.clear(false);
	}

    /**
     * Array indexing operator
     * @param index Index to retrieve
     * @return The address of the object at the given index
     */
    inline Obj& operator[](unsigned int index)
	{ return m_data[index]; }

    /**
     * Array indexing operator
     * @param index Index yo retrieve
     * @return Const address of the object at the given index
     */
    inline const Obj& operator[](unsigned int index) const
	{ return m_data[index]; }

    /**
     * Assignment operator
     * @param other Array to set from
     * @return This array's address
     */
    inline SigProcVector<Obj>& operator=(const SigProcVector<Obj>& other) {
	    assign(other.length());
	    copy(other.data(),other.length());
	    return *this;
	}

private:
     Obj* m_data;
     unsigned int m_length;
};


typedef SigProcVector<Complex> ComplexVector;


/**
 * Helpful class which holds a vector of complex values
 * @short A Complex array
 */
class ComplexArray : public ComplexVector, public GenObject
{
    YCLASS(ComplexArray,GenObject)
public:
    /**
     * Constructor
     */
    inline ComplexArray()
	{}

    /**
     * Constructor from Complex array
     * @param buf Source array
     * @param inLen Input array size
     * @param len Optional array size, 0 to use inLen
     */
    inline ComplexArray(unsigned int len)
	: ComplexVector(len)
	{}

    /**
     * Constructor from Complex array
     * @param ptr Source buffer
     * @param n Elements to copy from source buffer
     * @param len Optional vector length, 0 to use source buffer length
     */
    inline ComplexArray(const Complex* ptr, unsigned int n, unsigned int len = 0)
	: ComplexVector(ptr,n,len)
	{}

    /**
     * Constructor from int16 array
     * @param buf Source array
     * @param len The array size
     */
    inline ComplexArray(const int16_t* buf, unsigned int len)
	: ComplexVector((len + 1) / 2)
	{ setInt16(buf,len); }

    /**
     * Set array elements from 16 bit integers array
     * @param buf The source buffer
     * @param len Buffer length in 16 integer bit elements
     * @param offs Optional offset in our array to start copy
     */
    inline void setInt16(const int16_t* buf, unsigned int len, unsigned int offs = 0)
	{ Complex::setInt16(data(),length(),buf,len,offs); }

    /**
     * Sum this array with another one.
     * @param ca The complex array to sum with.
     */
    inline void sum(const ComplexArray& ca)
	{ Complex::sum(data(),length(),ca.data(),ca.length()); }

    /**
     * Multiply this array with another one.
     * @param ca The complex array to multiply with.
     */
    inline void multiply(const ComplexArray& ca)
	{ Complex::multiply(data(),length(),ca.data(),ca.length()); }

    /**
     * Helper method to dump all the data from the array
     * @param dest The destination string.
     * @param sep Optionaly a separator between values.
     */
    inline void dump(String& dest, const char* sep = " ") const
	{ Complex::dump(dest,data(),length(),sep); }
};


typedef SigProcVector<float> FloatVector;


class SignalProcessing
{
public:

    /**
     * Constructor
     */
    SignalProcessing()
	: m_oversample(0), m_arfcns(0), m_sinusoids(0)
	{}

    /**
     * Destructor
     */
    ~SignalProcessing();

    /**
     * Initialize internal data
     * @param oversample The oversampling value of the transceiver
     * @param arfcns The number of arfcns used by the transceiver
     */
    void initialize(unsigned int oversample, unsigned int arfcns);

    /**
     * Generate frequency shift complex array
     * Formula used: ret[i] = e^(-sqrt(-1) * i * PI / 2 * oversample)
     * @param out The output complex array
     * @param oversample The oversample value used to generate the frequency shift
     */
    static void fillFrequencyShift(ComplexArray* out, unsigned int oversample);

    /**
     * Generate Laurent pulse aproximation complex array
     * Formula used: ret[i] = 0.96  * e ^ g(i) / oversample
     * Where g(i) = -1.138 * ((i - oversample * 3) / oversample) ^ 2 -
     *        0.527 * ((i - oversample * 3) / oversample) ^ 4
     * @param out The output complex array
     * @param oversample The oversample value used to generate the pulse
     */
    static void fillLaurentPulseAproximation(ComplexArray* out, unsigned int oversample);

    /**
     * Calculate the convolution of two arrays of complex numbers.
     * Algorithm used: ret[x] = SUM (for i = 0 to n) from (ca1[x + i] * ca2[n - i])
     * Where: n represents the length of ca2 - 1
     * @param ca1 The first complex array
     * @param ca2 The second complex array
     * @return The convolved array.
     */
    static ComplexArray* getConvolution(const ComplexArray& ca1, const ComplexArray& ca2);

    /**
     * Modulate the given data.
     * @param inData Data to modulate.
     * @param inDataLength Length of the data to modulate.
     * @return The modulated data or 0 on error.
     */
    inline ComplexArray* modulate(unsigned char* inData, unsigned int inDataLength) const
	{ return modulate(inData,inDataLength,m_oversample,&m_freqencyShift,&m_laurentPulseAproximation); }

    /**
     * Modulate the given data with the specified parameters
     * @param inData The data to modulate
     * @param inDataLength Length of the data to modulate.
     * @param oversample The oversample rate used
     * @
     * @return The modulated data or 0 on error.
     */
    static ComplexArray* modulate(unsigned char* inData, unsigned int inDataLength, 
	unsigned int oversample, const ComplexArray* frequencyShift,
	const ComplexArray* laurentPulseAproximation);

    /**
     * Get the precalculated sinusoid array for the given arfcn
     * @param arfcn The arfcn number of the sinusoid array
     * @return The sinusoid array or 0.
     */
    const ComplexArray* getSinusoid(unsigned int arfcn) const;

    /**
     * Generate a frequency shifting vector
     * Each array element at index n will be set with e ^ (-j * n * val)
     *   where -j is the complex number (0,-1)
     * @param data Destination vector
     * @param val Frequency shifting value
     * @param setLen Optional vector length to set, 0 to use current array length
     */
    static void setFreqShifting(ComplexVector* data, float val, unsigned int setLen = 0);

    /**
     * Compute the power level from Complex array middle
     * @param data Array data start
     * @param len Array length
     * @param middle Middle number of elements to use
     * @param param Parameter to multiply with the elements sum result
     * @return Power value
     */
    static inline float computePower(const Complex* data, unsigned int len,
	unsigned int middle, float param = 1.0) {
	    if (data && len && middle <= len)
		return param * Complex::sumMulConj(data + (len - middle) / 2,middle);
	    return 0;
	}

    /**
     * Compute the power level from float array middle elements real part value
     * @param data Array data start
     * @param len Array length
     * @param middle Middle number of elements to use
     * @param param Parameter to multiply with the elements sum result
     * @return Power value
     */
    static float computePower(const float* data, unsigned int len,
	unsigned int middle, float param = 1.0);
    
    inline unsigned int getModulatedLength() const
	{ 
	    unsigned int modlen = m_oversample * BITS_PER_TIMESLOT;
	    return (m_oversample % 4 != 0) ? modlen++ : modlen;
	}

protected:
    void buildSinusoids();
private:
    unsigned int m_oversample;
    unsigned int m_arfcns;
    ComplexArray* m_sinusoids;
    ComplexArray m_freqencyShift;
    ComplexArray m_laurentPulseAproximation;
};

}; // namespace TelEngine

#endif // SIGPROC_H

/* vi: set ts=8 sw=4 sts=4 noet: */
