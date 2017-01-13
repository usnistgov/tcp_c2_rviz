

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
 */

//#pragma message "Compiling " __FILE__ 
#pragma once

#define _USE_MATH_DEFINES
#include <math.h>       

struct CMath {
    static const double E = M_E; // returns Euler's number
    static const double PI = M_PI; // returns PI
    static const double SQRT2 = M_SQRT2; // returns the square root of 2
    static const double SQRT1_2 = M_SQRT1_2; // returns the square root of 1/2
    static const double LN2 = M_LN2; // returns the natural logarithm of 2
    static const double LN10 = M_LN10; // returns the natural logarithm of 10
    static const double LOG2E = M_LOG2E; // returns base 2 logarithm of E
    static const double LOG10E = M_LOG10E; // returns base 10 logarithm of E 

    CMath() {
        // More random
        //srand((unsigned int) time (NULL)); //activates the generator
        srand(0); //activates the generator

    }

    /**The Math.random function returns a floating-point, pseudo-random number in the range [ 0, 1) 
     * that is, from 0 (inclusive) up to but not including 1 (exclusive), which you can then scale 
     * to your desired range.*/
    double random() {
        double r = ((double) rand() / (RAND_MAX)); //gives a random from 0 to 1
        return r;
    }

    template<typename T>
    T pow(T x, T y) {
        return ::pow(x, y);
    }

    template<typename T>
    T sqrt(T x) {
        return ::sqrt(x);
    }

    template<typename T>
    T abs(T x) {
        return ::fabs(x);
    }

    template<typename T>
    T sin(T x) {
        return ::sin(x);
    }

    template<typename T>
    T cos(T x) {
        return ::cos(x);
    }

    template<typename T>
    T min(T x, T y) {
        return (x < y) ?  x :  y;
    }

    template<typename T>
    T max(T x, T y) {
        return (x < y) ?  y :  x;
    }

    template<typename T>
    T ceil(T x) {
        return ::ceil(x);
    }

    template<typename T>
    T floor(T d) {
        return ::floor(d);
    }

    template<typename T>
    T round(T d) {
        return ::round(d);
    }
};

extern CMath Math;