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


#ifndef COLORIMAGE_H_
#define COLORIMAGE_H_


namespace chisel
{

    template <class DataType = uint8_t> struct Color
    {
            DataType red;
            DataType green;
            DataType blue;
            DataType alpha;
    };

    template <class DataType = uint8_t> class ColorImage
    {
        public:
            ColorImage() :
                data(nullptr), width(-1), height(-1), numChannels(0)
            {

            }
            ColorImage(int w, int h, size_t channels) :
                data(nullptr), width(w), height(h), numChannels(channels)
            {
                data = new DataType[width * height * numChannels];
            }

            virtual ~ColorImage()
            {
                if(data != nullptr)
                {
                    delete [] data;
                    data = nullptr;
                }
            }

            inline int Index(int row, int col, int channel) const
            {
                return (col + row * width) * numChannels + channel;
            }

            // this can slow down!
            inline void At(int row, int col, Color<DataType>* colorOut) const
            {
                const int index = Index(row, col, 0);

                switch (numChannels)
                {
                    // Mono
                    case 1:
                        colorOut->red = data[index];
                        colorOut->green = colorOut->red;
                        colorOut->blue = colorOut->red;
                        colorOut->alpha = colorOut->red;
                        break;
                    // Two Channel
                    case 2:
                        colorOut->red = data[index];
                        colorOut->green = data[index + 1];
                        colorOut->blue = colorOut->green;
                        colorOut->alpha = colorOut->green;
                        break;
                    // BGR
                    case 3:
                        colorOut->red = data[index + 2];
                        colorOut->green = data[index + 1];
                        colorOut->blue = data[index];
                        colorOut->alpha = colorOut->red;
                        break;
                   // BGRA
                    case 4:
                        colorOut->red = data[index + 2];
                        colorOut->green = data[index + 1];
                        colorOut->blue = data[index];
                        colorOut->alpha = data[index + 3];
                        break;
                }
            }

            // we already know the image is BGR
            inline void AtBGR(int row, int col, Color<DataType>* colorOut) const
            {
                //const int index = Index(row, col, 0);
                
                int index = (col + row * width) * numChannels;

                colorOut->blue  = data[index++];                
                colorOut->green = data[index++];
                colorOut->red   = data[index];
               
                colorOut->alpha = 1;
            }

            inline const DataType& At(int row, int col, int channel) const
            {
                const int index = Index(row, col, channel);
                return data[index];
            }

            inline DataType& AtMutable(int row, int col, int channel) const
            {
                return data[Index(row, col, channel)];
            }

            inline bool IsInside(int row, int col) const
            {
                return row >= 0 && row < width && col >= 0 && col < height;
            }

            inline const DataType* GetData() const { return data; }
            inline DataType* GetMutableData() { return data; }
            inline void SetData(DataType* d) { data = d; }
            inline int GetWidth() const { return width; }
            inline int GetHeight() const { return height; }
            inline void SetWidth(int w) { width = w; }
            inline void SetHeight(int h) { height = h; }
            inline void SetNumChannels(size_t nc) { numChannels = nc; }

            inline size_t GetNumChannels() const { return numChannels; }

        protected:
            DataType* data;
            int width;
            int height;
            size_t numChannels;
    };

} // namespace chisel



#endif // COLORIMAGE_H_ 
