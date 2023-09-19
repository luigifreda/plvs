// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <open_chisel/io/PLY.h>

#include <iostream>
#include <fstream>

namespace chisel
{
    bool SaveMeshPLYASCII(const std::string& fileName, const chisel::MeshConstPtr& mesh)
    {
        std::ofstream stream(fileName.c_str());

        if (!stream)
        {
            return false;
        }

        size_t numPoints = mesh->vertices.size();
        stream << "ply" << std::endl;
        stream << "format ascii 1.0" << std::endl;
        stream << "element vertex " << numPoints << std::endl;
        stream << "property float x" << std::endl;
        stream << "property float y" << std::endl;
        stream << "property float z" << std::endl;
        if (mesh->HasColors())
        {
            stream << "property uchar red" << std::endl;
            stream << "property uchar green" << std::endl;
            stream << "property uchar blue" << std::endl;
        }
        stream << "element face " << numPoints / 3 <<  std::endl;
        stream << "property list uchar int vertex_index" << std::endl;
        stream << "end_header" << std::endl;

        size_t vert_idx = 0;
        for (const Vec3& vert : mesh->vertices)
        {
            stream << vert(0) << " " << vert(1) << " " << vert(2);

            if (mesh->HasColors())
            {
                const Vec3& color = mesh->colors[vert_idx];
                int r = static_cast<int>(color(0) * 255.0f);
                int g = static_cast<int>(color(1) * 255.0f);
                int b = static_cast<int>(color(2) * 255.0f);

                stream << " " << r << " " << g << " " << b;
            }

            stream << std::endl;
            vert_idx++;
        }

        for (size_t i = 0; i < mesh->indices.size(); i+=3)
        {
            stream << "3 ";

            for(int j = 0; j < 3; j++)
            {
                stream << mesh->indices.at(i + j) << " ";
            }

            stream << std::endl;
        }


    }
}
