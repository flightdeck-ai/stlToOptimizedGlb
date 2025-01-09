//
// Created by simon on 09.01.25.
//

#include "converter.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <tiny_gltf.h>
#include <stdexcept>
#include <fstream>
#include <cmath>

// Constructor
Converter::Converter(const std::string& stlFilePath) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(stlFilePath, aiProcess_Triangulate);
    if (!scene || !scene->HasMeshes()) {
        throw std::runtime_error("Failed to load STL file.");
    }

    const aiMesh* mesh = scene->mMeshes[0];
    for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
        const aiFace& face = mesh->mFaces[i];
        if (face.mNumIndices == 3) {
            for (unsigned int j = 0; j < 3; ++j) {
                const aiVector3D& vertex = mesh->mVertices[face.mIndices[j]];
                vertices.push_back({{vertex.x, vertex.y, vertex.z}, {1.0f, 1.0f, 1.0f}});
            }
        }
    }
}

// Bounding coordinates calculation
std::pair<std::array<float, 3>, std::array<float, 3>> Converter::boundingCoords() const {
    std::array<float, 3> min{FLT_MAX, FLT_MAX, FLT_MAX};
    std::array<float, 3> max{-FLT_MAX, -FLT_MAX, -FLT_MAX};

    for (const auto& vertex : vertices) {
        for (int i = 0; i < 3; ++i) {
            min[i] = std::min(min[i], vertex.position[i]);
            max[i] = std::max(max[i], vertex.position[i]);
        }
    }
    return {min, max};
}

// Align size to multiple of 4
void Converter::alignToMultipleOfFour(size_t& size) const {
    size = (size + 3) & ~3;
}

// Convert vertices into a padded byte vector
std::vector<uint8_t> Converter::toPaddedByteVector() const {
    const size_t byteLength = vertices.size() * sizeof(Vertex);
    std::vector<uint8_t> data(reinterpret_cast<const uint8_t*>(vertices.data()),
                              reinterpret_cast<const uint8_t*>(vertices.data()) + byteLength);
    while (data.size() % 4 != 0) {
        data.push_back(0);
    }
    return data;
}

// Convert to GLB
std::vector<uint8_t> Converter::convertToGLB() {
    auto [min, max] = boundingCoords();

    tinygltf::Model model;
    tinygltf::Scene scene;
    model.scenes.push_back(scene);
    model.defaultScene = 0;

    size_t bufferLength = vertices.size() * sizeof(Vertex);
    tinygltf::Buffer buffer;
    buffer.data = toPaddedByteVector();
    buffer.byteLength = bufferLength;
    model.buffers.push_back(buffer);

    tinygltf::BufferView bufferView;
    bufferView.buffer = 0;
    bufferView.byteOffset = 0;
    bufferView.byteLength = bufferLength;
    bufferView.byteStride = sizeof(Vertex);
    bufferView.target = TINYGLTF_TARGET_ARRAY_BUFFER;
    model.bufferViews.push_back(bufferView);

    tinygltf::Accessor positionAccessor;
    positionAccessor.bufferView = 0;
    positionAccessor.byteOffset = 0;
    positionAccessor.count = vertices.size();
    positionAccessor.type = TINYGLTF_TYPE_VEC3;
    positionAccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
    positionAccessor.minValues = {min[0], min[1], min[2]};
    positionAccessor.maxValues = {max[0], max[1], max[2]};
    model.accessors.push_back(positionAccessor);

    tinygltf::Accessor colorAccessor;
    colorAccessor.bufferView = 0;
    colorAccessor.byteOffset = 3 * sizeof(float);
    colorAccessor.count = vertices.size();
    colorAccessor.type = TINYGLTF_TYPE_VEC3;
    colorAccessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
    model.accessors.push_back(colorAccessor);

    tinygltf::Primitive primitive;
    primitive.attributes["POSITION"] = 0;
    primitive.attributes["COLOR_0"] = 1;
    primitive.mode = TINYGLTF_MODE_TRIANGLES;

    tinygltf::Mesh mesh;
    mesh.primitives.push_back(primitive);
    model.meshes.push_back(mesh);

    tinygltf::Node node;
    node.mesh = 0;
    model.nodes.push_back(node);

    tinygltf::Scene gltfScene;
    gltfScene.nodes.push_back(0);
    model.scenes.push_back(gltfScene);

    tinygltf::TinyGLTF gltfWriter;
    std::vector<uint8_t> outputBuffer;
    if (!gltfWriter.WriteGltfSceneToBuffer(&model, &outputBuffer, true, true, true, true)) {
        throw std::runtime_error("Failed to write GLB file.");
    }

    return outputBuffer;
}