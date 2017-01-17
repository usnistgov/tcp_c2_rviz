

#pragma once

#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <boost/thread.hpp>

typedef sensor_msgs::JointState_<std::allocator<void> > JointState;

/*!
 * \brief StrFormat  accepts a traditional C format string and expects parameter to follow on calling stack and will
 * produce a string from it.
 * \param fmt is the C format string.
 */
inline std::string StrFormat(const char *fmt, ...) {
    va_list argptr;
    va_start(argptr, fmt);
    int m;
    int n = (int) strlen(fmt) + 1028;
    std::string tmp(n, '0');
    while ((m = vsnprintf(&tmp[0], n - 1, fmt, argptr)) < 0) {
        n = n + 1028;
        tmp.resize(n, '0');
    }
    va_end(argptr);
    return tmp.substr(0, m);
}

/*!
 * \brief sleep milliseconds. Equivalent to Sleep in windows.
 * \param ms number of milliseconds to sleep
 */
inline void Sleep(unsigned int ms) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

inline std::vector<std::string> Split(const std::string &text, char sep, bool bKeepEmpty = false) {
    std::vector<std::string> tokens;
    std::size_t start = 0, end = 0;
    std::string token;
    while ((end = text.find(sep, start)) != std::string::npos) {
        token = text.substr(start, end - start);
//        if (!token.empty())
            tokens.push_back(token);
        start = end + 1;
    }
    token = text.substr(start);
//    if (!token.empty())
        tokens.push_back(token);
    return tokens;
}

inline std::string LeftTrim(std::string str) {
    size_t startpos = str.find_first_not_of(" \t\r\n");

    if (std::string::npos != startpos) {
        str = str.substr(startpos);
    }
    return str;
}
// trim from end

inline std::string RightTrim(std::string str, std::string trim = " \t\r\n") {
    size_t endpos = str.find_last_not_of(trim);

    if (std::string::npos != endpos) {
        str = str.substr(0, endpos + 1);
    }
    return str;
}
// trim from both ends

inline std::string Trim(std::string s) {
    return LeftTrim(RightTrim(s));
}

struct randfunctor {

    randfunctor(double v) : val(v) {
    }

    double operator()() const {
        return (rand() / (double) RAND_MAX)*val;
    }
private:
    double val;
};

inline std::vector<double> GenRandomVector(size_t n, double min, double max) {
    std::vector<double> dvec;
    double range = fabs(min) + fabs(max);
    std::generate_n(std::back_inserter(dvec), n, randfunctor(range));
    for(size_t i=0; i< dvec.size(); i++)
        dvec[i]=dvec[i]-fabs(min);
    return dvec;
}

