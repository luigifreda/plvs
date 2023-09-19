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

#ifndef DEPTHIMAGE_H_
#define DEPTHIMAGE_H_

#include <memory>
#include <Eigen/Core>

#include <open_chisel/geometry/Interpolate.h>

namespace chisel
{

    template <class DataType = float> class DepthImage
    {
        public:
            DepthImage() :
                data(nullptr), width(-1), height(-1)
            {

            }

            DepthImage(int w, int h) :
                data(nullptr), width(w), height(h)
            {
                data = new DataType[width * height];
            }

            virtual ~DepthImage()
            {
                delete [] data;
                data = nullptr;
            }

            inline int Index(const int& row, const int& col) const
            {
                return col + row * width;
            }

            inline float BilinearInterpolateDepth(float x, float y) const
            {
                int gxi = static_cast<int>(x);
                int gyi = static_cast<int>(y);
                float c00 = DepthAt(gyi, gxi);
                float c10 = DepthAt(gyi, gxi + 1);
                float c01 = DepthAt(gyi + 1, gxi);
                float c11 = DepthAt(gyi + 1, gxi + 1);
                return BilinearInterpolate(c00, c10, c01, c11, x - gxi, y - gyi);
            }

            inline float DepthAt(const int& row, const int& col) const
            {
                const DataType& d = At(row, col);
                return static_cast<float>(d);
            }

            inline const DataType& At(const int& row, const int& col) const
            {
                return data[Index(row, col)];
            }

            inline DataType& AtMutable(const int& row, const int& col)
            {
                return data[Index(row, col)];
            }

            inline bool IsInside(const int& row, const int& col) const
            {
                return row >= 0 && row < width && col >= 0 && col < height;
            }

            void GetStats(DataType& minimum, DataType& maximum, DataType& mean, DataType invalidValid=0) const
            {
                int numPixels = width * height;
                minimum = std::numeric_limits<DataType>::max();
                maximum = -std::numeric_limits<DataType>::max();
                mean = static_cast<DataType>(0);

                for (int i = 0; i < numPixels; i++)
                {
                    const DataType& pixel = data[i];

                    if (pixel == invalidValid) continue;
                    if (std::isnan(pixel)) continue;

                    minimum = std::min(pixel, minimum);
                    maximum = std::max(pixel, maximum);
                    mean += pixel;
                }
                mean /= numPixels;
            }

            inline const DataType* GetData() const { return data; }
            inline DataType* GetMutableData() { return data; }
            inline void SetData(DataType* d) { data = d; }
            inline int GetWidth() const { return width; }
            inline int GetHeight() const { return height; }
            inline void SetWidth(int w) { width = w; }
            inline void SetHeight(int h) { height = h; }

        protected:
            DataType* data;
            int width;
            int height;
    };



} // namespace chisel 

#endif // DEPTHIMAGE_H_ 
