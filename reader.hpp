#ifndef READER_HPP
#define READER_HPP

#include <cctype>
#include <fstream>
#include <sstream>
#include <string>

class Reader {
public:
    Reader(std::istream *i);
    bool expect(std::string s);
    bool peek(std::string s);
    void swallowWhitespace();
    void swallowLine();
    bool readFloat(float &f);
    bool readInt(int &i);
    bool readToken(std::string &s);
    bool good();
    bool readLine(std::string &line);
    bool upcomingInt();
protected:
    bool floatChar(char c);
    bool intChar(char c);
    bool tokenChar(char c);
    std::istream *in;
};
 
inline Reader::Reader(std::istream *i) {
    in = i;
}

inline bool Reader::expect(std::string s) {
    swallowWhitespace();
    if (s.size() == 0) {
        return true;
    }
    std::string swallowed;
    char c;
    in->get(c);
    swallowed += c;
    while ((*in) && swallowed.size() < s.size()
           && swallowed == s.substr(0, swallowed.size())) {
        in->get(c);
        swallowed += c;
    }
    if (swallowed == s) {
        return true;
    }
    else {
        std::string::reverse_iterator it;
        for (it = swallowed.rbegin(); it != swallowed.rend(); ++it) {
            in->putback(*it);
        }
        return false;
    }
}

inline bool Reader::peek(std::string s) {
    swallowWhitespace();
    bool retval = false;
    if (s.size() == 0) {
        return true;
    }
    std::string swallowed;
    char c;
    in->get(c);
    swallowed += c;
    while ((*in) && swallowed.size() < s.size() && swallowed == s.substr(0, swallowed.size())) {
        in->get(c);
        swallowed += c;
    }
    if (swallowed == s) {
        retval = true;
    }
    else {
        retval = false;
    }
    std::string::reverse_iterator it;
    for (it = swallowed.rbegin(); it != swallowed.rend(); ++it) {
        in->putback(*it);
    }
    return retval;
}

inline void Reader::swallowWhitespace() {
    char c;
    in->get(c);
    while ((*in) && std::isspace(c)) {
        in->get(c);
    }
    in->putback(c);
}

inline void Reader::swallowLine() {
    std::string placeholder;
    std::getline(*in, placeholder);
    swallowWhitespace();
}

inline bool Reader::readFloat(float &f) {
    swallowWhitespace();
    std::string accum;
    char c;
    in->get(c);
    while ((*in) && floatChar(c)) {
        accum += c;
        in->get(c);
    }
    in->putback(c);
    std::stringstream ss(accum);
    ss >> f;
    return true;
}

inline bool Reader::readInt(int &i) {
    swallowWhitespace();
    std::string accum;
    char c;
    in->get(c);
    while ((*in) && intChar(c)) {
        accum += c;
        in->get(c);
    }
    in->putback(c);
    std::stringstream ss(accum);
    ss >> i;
    return true;  
}

inline bool Reader::readToken(std::string &s) {
    swallowWhitespace();
    std::string accum;
    char c;
    in->get(c);
    while ((*in) && tokenChar(c)) {
        accum += c;
        in->get(c);
    }
    in->putback(c);
    s = accum;
    return true;
}

inline bool Reader::good() {
    return (bool)(*in);
}

inline bool Reader::readLine(std::string &line) {
    getline(*in, line);
    return true;
}

inline bool Reader::upcomingInt() {
    swallowWhitespace();
    char c;
    in->get(c);
    in->putback(c);
    return intChar(c);
}

inline bool Reader::floatChar(char c) {
    return ( c == 'e'
             || (c >= '0' && c <= '9')
             || c == '.'
             || c == '+'
             || c == '-'
             );
}

inline bool Reader::intChar(char c) {
    return ( (c >= '0' && c <= '9') || c == '-');
}

inline bool Reader::tokenChar(char c) {
    return !std::isspace(c);
}

#endif
