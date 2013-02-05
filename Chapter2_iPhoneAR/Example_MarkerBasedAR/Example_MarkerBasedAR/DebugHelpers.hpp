#ifndef DEBUG_HELPERS_HPP
#define DEBUG_HELPERS_HPP

#include <string>
#include <sstream>

template <typename T>
std::string ToString(const T& value)
{
    std::ostringstream stream;
    stream << value;
    return stream.str();
}

namespace cv
{
    inline void showAndSave(std::string name, const cv::Mat& m)
    {
        cv::imshow(name, m);
        cv::imwrite(name + ".png", m);
    }
}

#endif