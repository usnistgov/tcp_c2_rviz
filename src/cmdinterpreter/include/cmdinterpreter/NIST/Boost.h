#pragma once

/*
DISCLAIMER:
This software was produced by the National Institute of Standards
and Technology (NIST), an agency of the U.S. government, and by statute is
not subject to copyright in the United States.  Recipients of this software
assume all responsibility associated with its operation, modification,
maintenance, and subsequent redistribution.

See NIST Administration Manual 4.09.07 b and Appendix I.
*/

#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
namespace pt = boost::property_tree;
/**
   Turn string s into a vector of types T using separator
 */
template<class T>
void tokenizeV(const std::string &s,
          std::vector<T> &o, std::string separator)
{
  typedef boost::tokenizer<boost::char_separator<char> > tok_t;
    boost::char_separator<char> sep(separator.c_str());
    tok_t tok(s, sep);
    for (tok_t::iterator j (tok.begin());
      j != tok.end();
      ++j)
  {
    std::string f(*j);
    boost::trim(f);
    o.push_back(boost::lexical_cast<T>(f));
  }
}


template <typename T>
inline std::vector<T> GetTypes(pt::ptree &root, std::string childname) {
    std::vector<T> ts;

    BOOST_FOREACH(pt::ptree::value_type &v2,
            root.get_child(childname)) {
        ts.push_back(v2.second.get_value<T>());
    }
    return ts;
}

template <typename T>
inline std::vector<T> GetIniTypes(pt::ptree &root, std::string childname) {
    std::vector<T> ts;
    std::string str = root.get<std::string>(childname);
#if 0
    boost::split(ts, str, boost::is_any_of(","), boost::token_compress_on);
#endif
    tokenizeV(str,ts,",");
    return ts;
}

#define BOOST_LOG_DYN_LINK 1
#include <boost/log/expressions.hpp>
#include <boost/log/sources/global_logger_storage.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup.hpp>

#define LOG_TRACE  BOOST_LOG_SEV(my_logger::get(), boost::log::trivial::trace)
#define LOG_DEBUG  BOOST_LOG_SEV(my_logger::get(), boost::log::trivial::debug)
#define LOG_INFO  BOOST_LOG_SEV(my_logger::get(), boost::log::trivial::info)
#define LOG_WARN  BOOST_LOG_SEV(my_logger::get(), boost::log::trivial::warning)
#define LOG_ERROR BOOST_LOG_SEV(my_logger::get(), boost::log::trivial::error)
#define LOG_FATAL  BOOST_LOG_SEV(my_logger::get(), boost::log::trivial::fatal)

extern std::string boostlogfile ;                       
extern boost::log::trivial::severity_level boostloglevel;

//Narrow-char thread-safe logger.
typedef boost::log::sources::severity_logger_mt<boost::log::trivial::severity_level> logger_t;

//declares a global logger with a custom initialization
BOOST_LOG_GLOBAL_LOGGER(my_logger, logger_t)
extern void set_log_file(const char* filename);