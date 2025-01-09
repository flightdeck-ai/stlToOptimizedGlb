//
// Created by simon on 09.01.25.
//

#ifndef CONVERTER_H
#define CONVERTER_H

#include <array>
#include <cstdint>
#include <vector>
#include <string>

// Class to convert STL to GLB
class Converter {
public:
    explicit Converter(const std::string& stlFilePath);
    std::vector<uint8_t> convertToGLB();

private:
    struct Vertex {
        std::array<float, 3> position;
        std::array<float, 3> color;
    };

    std::vector<Vertex> vertices;

    std::pair<std::array<float, 3>, std::array<float, 3>> boundingCoords() const;
    std::vector<uint8_t> toPaddedByteVector() const;
    void alignToMultipleOfFour(size_t& size) const;
};

#endif //CONVERTER_H
