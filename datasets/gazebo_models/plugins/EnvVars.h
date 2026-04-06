#ifndef envvars_h
#define envvars_h

#include <algorithm>
#include <any>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <unordered_map>
#include <vector>


using namespace std;



std::string ExpandEnvVars(const std::string& str) {
   std::string result;
   size_t pos = 0;

   while (pos < str.length()) {
      if (str[pos] == '$') {
         size_t start = pos + 1;
         size_t end = start;

         // Handle ${VAR} format
         if (start < str.length() && str[start] == '{') {
            end = str.find('}', start);
            if (end != std::string::npos) {
               std::string varName = str.substr(start + 1, end - start - 1);
               const char* varValue = getenv(varName.c_str());
               if (varValue) {
                  result += varValue;
               }
               pos = end + 1;
               continue;
            }
         }

         // Handle $VAR format
         while (end < str.length() && (isalnum(str[end]) || str[end] == '_')) {
            ++end;
         }
         std::string varName = str.substr(start, end - start);
         const char* varValue = getenv(varName.c_str());
         if (varValue) {
            result += varValue;
         }
         pos = end;
      } else {
         result += str[pos];
         ++pos;
      }
   }
   return result;
}

#endif
