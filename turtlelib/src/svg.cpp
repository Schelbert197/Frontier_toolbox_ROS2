#include <iostream>
#include <cmath>
#include <fstream>
#include <iosfwd>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"

namespace turtlelib{

    Svg::Svg() : filename("new_file"){
        open_svg(filename);
        define_arrow();
    }

    Svg::Svg(std::string f_name) : filename(f_name){
        open_svg(filename);
        define_arrow();
    }

    void Svg::open_svg(const std::string & filename){
        outputFile.open(filename);

        if(!outputFile.is_open()){
            std::cerr << "Error: Cannot open file" << std::endl;
        }

        outputFile << "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">" << std::endl;

    }

    void Svg::define_arrow(){
        outputFile << "<defs>" << std::endl;
        outputFile << "<marker style=\"overflow:visible\" id=\"Arrow1Sstart\" refX=\"0.0\" refY=\"0.0\" orient=\"auto\"> <path transform=\"scale(0.2) translate(6,0)\" style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\" d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"/></marker>" << std::endl;
        outputFile << "</defs>" << std::endl;
    }

    void Svg::draw_circle(const CircleSVG circ){
        double new_cx = (circ.cx * 96) + 408;
        double new_cy = (circ.cy * -96) + 528;
        outputFile << "<circle cx=\"" << new_cx << "\" cy=\"" << new_cy << "\" r=\"" << circ.r  << "\" stroke=\"" << circ.stroke << "\" fill=\"" << circ.fill << "\" stroke-width=\"" << circ.stroke_width << "\" />" << std::endl;
    }

    void Svg::draw_line(const LineSVG line){
        double new_x1 = (line.x1 * 96) + 408;
        double new_x2 = (line.x2 * 96) + 408;
        double new_y1 = (line.y1 * -96) + 528;
        double new_y2 = (line.y2 * -96) + 528;
        outputFile << "<line x1=\"" << new_x1 << "\" x2=\"" << new_x2 << "\" y1=\"" << new_y1 << "\" y2=\"" << new_y2 << "\" stroke=\"" << line.stroke << "\" stroke-width=\"" << line.stroke_width << "\" marker-start=\"" << line.marker_start << "\" />" << std::endl;
    }

    void Svg::draw_text(const double x, const double y, const std::string words){
        double new_x = (x * 96) + 408;
        double new_y = (y * -96) + 528;
        outputFile << "<text x=\"" << new_x << "\" y=\"" << new_y << "\">{" << words << "}</text>" << std::endl;
    }

    void Svg::draw_frame(const Transform2D tf, const std::string frame_name){
        Point2D x_axis{1.0,0.0}, y_axis{0.0,1.0}, new_point_x, new_point_y;
        LineSVG x_line, y_line;
        double text_x = tf.translation().x + 0.02;
        double text_y = tf.translation().y + 0.02;
        new_point_x = tf(x_axis);
        new_point_y = tf(y_axis);
        x_line.x1 = new_point_x.x;
        x_line.y1 = new_point_x.y;
        x_line.x2 = tf.translation().x;
        x_line.y2 = tf.translation().y;
        x_line.stroke = "red";
        y_line.x1 = new_point_y.x;
        y_line.y1 = new_point_y.y;
        y_line.x2 = tf.translation().x;
        y_line.y2 = tf.translation().y;
        y_line.stroke = "green";
        outputFile << "<g>" << std::endl;
        draw_line(x_line);
        draw_line(y_line);
        draw_text(text_x, text_y, frame_name);
        outputFile << "</g>" << std::endl;
    }

    void Svg::close_svg(){
        outputFile << "</svg>" << std::endl;
        outputFile.close();
    }

}