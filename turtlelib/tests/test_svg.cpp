#include <iostream>
#include <cmath>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <fstream>
#include <iosfwd>
#include "turtlelib/svg.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>

namespace turtlelib{
    TEST_CASE("svg fstream insertion for text", "[transform]"){ // Srikanth, Schelbert
        double x = 0.0, y= 0.0;
        Svg svg = Svg();
        std::string word = "{a}";
        std::string str1, str2 = "<text x=\"408\" y=\"528\">{a}</text>", str3;
        std::fstream fstr;
        svg.draw_text(x,y,word);
        fstr >> str1;
        fstr >> str3;
        REQUIRE(str3 == str2);
    }
}