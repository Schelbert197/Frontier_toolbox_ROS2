#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations visualized.


#include<iosfwd> // contains forward definitions for iostream objects
#include <fstream>
#include<string>

#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"

namespace turtlelib{

    /// \brief the circlular components for an SVG file
    struct CircleSVG{

        /// \brief the center x pos
        double cx = 0.0;

        /// \brief the center y pos
        double cy = 0.0;

        /// \brief the circle radius
        double r = 5.0;

        /// \brief the color of the line
        std::string stroke = "purple";

        /// \brief the fill color of the circle
        std::string fill = "purple";

        /// \brief the line width
        int stroke_width = 1;
    };

    /// \brief A line type object for svg setup
    struct LineSVG{

        /// \brief the start x pos
        double x1 = 0.0;

        /// \brief the start y pos
        double y1 = 0.0;

        /// \brief the end x pos
        double x2 = 0.0;

        /// \brief the end y pos
        double y2 = 0.0;

        /// \brief the color of the line
        std::string stroke = "purple";

        /// \brief the line width
        int stroke_width = 5;

        /// \brief the marker start
        std::string marker_start="url(#Arrow1Sstart)";
    };

/// \brief This class creates function that can be used to create
///        objects displayed in svg format.
///
///     \param filename (std::string): The filename for the svg file
///     \param outputFile (std::ofstream) : The output file object

    class Svg{
    private:
        std::string filename = "new_file";

        std::ofstream outputFile;
    public:
        /// \brief Create a new class instance as a constructor
        Svg();

        /// \brief Create an class instance with a given filename
        /// \param f_name - the fiilename object
        Svg(std::string f_name);

        /// \brief opens an svg file instance and an fstream instance
        /// \param filename - the fiilename object
        void open_svg(const std::string & filename);

        /// \brief defines an arrow to give a line object direction
        void define_arrow();

        /// \brief draws a circle object in the svg file
        /// \param circ - the circle object parameters
        void draw_circle(const CircleSVG circ);

        /// \brief draws a line object in the svg file
        /// \param line - the line object parameters
        void draw_line(const LineSVG line);

        /// \brief draws a frame object in the svg file
        /// \param tf - the transformation represented by the frame
        /// \param frame_name - the name of the frame the tf represents
        void draw_frame(const Transform2D tf, const std::string frame_name);

        /// \brief draws text on the page at a specific postion
        /// \param x - the x coordinate of the text
        /// \param y - the y coordinate of the text
        /// \param words - the text information
        void draw_text(const double x, const double y, const std::string words);

        /// \brief closes an svg file instance and an fstream instance
        void close_svg();
    };
}

#endif