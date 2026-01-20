/** @file */
/*
   std_lib_facilities.h
*/

/*
        simple "Programming: Principles and Practice using C++ (second edition)"
   course header to be used for the first few weeks. It provides the most common
   standard headers (in the global namespace) and minimal exception/error
   support.

        Students: please don't try to understand the details of headers just
   yet. All will be explained. This header is primarily used so that you don't
   have to understand every concept all at once.

        By Chapter 10, you don't need this file and after Chapter 21, you'll
   understand it

        Revised April 25, 2010: simple_error() added

        Revised November 25 2013: remove support for pre-C++11 compilers, use
   C++11: <chrono> Revised November 28 2013: add a few container algorithms
        Revised June 8 2014: added #ifndef to workaround Microsoft C++11
   weakness

   Revised for TDT4102, NTNU, 21 December 2018:
   - more std libs included.

   Revised on 30.08.2022:
   - Removed std::list and std::forward_list includes
   - Removed keep_window_open() functions
   - Use TDT4102 namespace
*/

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

// C++23
#if __cplusplus >= 202302L
#include <print>
#endif 

//------------------------------------------------------------------------------

using Unicode = long;

//------------------------------------------------------------------------------

// Create an empty namespace so we can use it
namespace TDT4102 {

}

using namespace std;
using namespace TDT4102;

template <class T>
string to_string(const T& t) {
    ostringstream os;
    os << t;
    return os.str();
}

struct Range_error : out_of_range {  // enhanced vector range error reporting
    int index;
    Range_error(int i) : out_of_range("Range error: " + to_string(i)), index(i) {}
};

struct Exit : runtime_error {
    Exit() : runtime_error("Exit") {}
};

// error() simply disguises throws:
[[noreturn]] inline void error(const string& s) {
    throw runtime_error(s);
}

[[noreturn]] inline void error(const string& s, const string& s2) {
    error(s + s2);
}

[[noreturn]] inline void error(const string& s, int i) {
    ostringstream os;
    os << s << ": " << i;
    error(os.str());
}

template <class T>
char* as_bytes(T& i)  // needed for binary I/O
{
    void* addr = &i;                  // get the address of the first byte
                                      // of memory used to store the object
    return static_cast<char*>(addr);  // treat that memory as bytes
}

// error function to be used (only) until error() is introduced in Chapter 5:
[[noreturn]] inline void simple_error(string s)  // write ``error: s and exit program
{
    cerr << "error: " << s << '\n';
    exit(1);
}

// make std::min() and std::max() accessible on systems with antisocial macros:
#undef min
#undef max

// run-time checked narrowing cast (type conversion).
template <class R, class A>
R narrow_cast(const A& a) {
    R r = R(a);
    if (A(r) != a)
        error(string("info loss"));
    return r;
}

// random number generators. See 24.7.
inline int randint(int min, int max) {
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    static default_random_engine ran{static_cast<unsigned int>(seed)};
    return uniform_int_distribution<>{min, max}(ran);
}

inline int randint(int max) {
    return randint(0, max);
}

// container algorithms. See 21.9.

template <typename C>
using Value_type = typename C::value_type;

template <typename C>
using Iterator = typename C::iterator;

template <typename C>
// requires Container<C>()
void sort(C& c) {
    std::sort(c.begin(), c.end());
}

template <typename C, typename Pred>
// requires Container<C>() && Binary_Predicate<Value_type<C>>()
void sort(C& c, Pred p) {
    std::sort(c.begin(), c.end(), p);
}

template <typename C, typename Val>
// requires Container<C>() && Equality_comparable<C,Val>()
Iterator<C> find(C& c, Val v) {
    return std::find(c.begin(), c.end(), v);
}

template <typename C, typename Pred>
// requires Container<C>() && Predicate<Pred,Value_type<C>>()
Iterator<C> find_if(C& c, Pred p) {
    return std::find_if(c.begin(), c.end(), p);
}
