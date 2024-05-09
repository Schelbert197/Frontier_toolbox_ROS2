#include <iostream>
#include <cmath>
#include <fstream>
#include <iosfwd>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"

using turtlelib::Svg;
using turtlelib::Transform2D;
using turtlelib::Point2D;
using turtlelib::Vector2D;
using turtlelib::LineSVG;
using turtlelib::CircleSVG;
int main(){
    Transform2D Tfa, Tab, Tba, Tbc, Tcb, Tac, Tca;
    Point2D Pa, Pb, Pc;
    Vector2D Vb, Vb_norm;
    LineSVG line1, line2, line3;
    CircleSVG P_a, P_b, P_c;


    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> Tab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> Tbc;
    Tba = Tab.inv();
    Tcb = Tbc.inv();
    Tac = Tab*Tbc;
    Tca = Tac.inv();
    std::cout << "T_{a,b}: " << Tab << "\n";
    std::cout << "T_{b,a}: " << Tba << "\n";
    std::cout << "T_{b,c}: " << Tbc << "\n";
    std::cout << "T_{c,b}: " << Tcb << "\n";
    std::cout << "T_{a,c}: " << Tac << "\n";
    std::cout << "T_{c,a}: " << Tca << std::endl;
    std::cout << "Enter point p_a:" << std::endl;
    std::cin >> Pa;
    Pb = Tba(Pa);
    Pc = Tca(Pa);
    std::cout << "p_a: " << Pa << "\n";
    std::cout << "p_b: " << Pb << "\n";
    std::cout << "p_c: " << Pc << "\n";
    std::cout << "Enter a vector v_b:" << std::endl;
    std::cin >> Vb;
    Vb_norm = normalize_vector(Vb);
    std::cout << "v_bhat:" << Vb_norm << "\n";
    std::cout << "v_a: " << Tab(Vb) << "\n";
    std::cout << "v_b: " << Vb << "\n";
    std::cout << "v_c: " << Tcb(Vb) << "\n";
    std::cout << "Enter a twist V_b:" << std::endl;
    Svg svg_obj("/tmp/frames.svg");
    P_a.cx = Pa.x; // draws the circles
    P_a.cy = Pa.y;
    P_b.cx = Pb.x;
    P_b.cy = Pb.y;
    P_b.fill = "brown";
    P_b.stroke = "brown";
    P_c.cx = Pc.x;
    P_c.cy = Pc.y;
    P_c.fill = "orange";
    P_c.stroke = "orange";
    svg_obj.draw_circle(P_a);
    svg_obj.draw_circle(P_b);
    svg_obj.draw_circle(P_c);
    line1.x1 = Tab(Vb_norm).x + Tab.translation().x; // draw the vectors
    line1.y1 = Tab(Vb_norm).y + Tab.translation().y;
    line1.x2 = Tab.translation().x;
    line1.y2 = Tab.translation().y;
    line1.stroke = "brown";
    // line2.x1 = Tca.translation().x;
    // line2.y1 = Tca.translation().y;
    line2.x1 = Tab(Vb).x;
    line2.y1 = Tab(Vb).y;
    line2.stroke = "purple";
    line3.x1 = Tac(Tcb(Vb)).x + Tac.translation().x;
    line3.y1 = Tac(Tcb(Vb)).y + Tac.translation().y;
    line3.x2 = Tac.translation().x;
    line3.y2 = Tac.translation().y;
    line3.stroke = "orange";
    svg_obj.draw_line(line1);
    svg_obj.draw_line(line2);
    svg_obj.draw_line(line3);
    svg_obj.draw_frame(Tfa, "a");
    svg_obj.draw_frame(Tab, "b");
    svg_obj.draw_frame(Tac, "c");
    svg_obj.close_svg();
    return 0;
}