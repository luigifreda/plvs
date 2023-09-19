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


#ifndef THREADING_H_
#define THREADING_H_

#include <cstddef>
#include <thread>
#include <algorithm>
#include <vector>

namespace chisel
{
    template<typename Iterator, class Function>
    void parallel_for(const Iterator& first, const Iterator& last, Function&& f, const int nthreads = 8, const int threshold = 1000)
    {
        const auto group = std::max(std::max(ptrdiff_t(1), ptrdiff_t(std::abs(threshold))), ((last-first))/std::abs(nthreads));
        std::vector<std::thread> threads;
        threads.reserve(nthreads);
        Iterator it = first;
        for (; it < last - group; it = std::min(it + group, last))
        {
            threads.push_back(std::thread([=,&f](){std::for_each(it, std::min(it+group, last), f);}));
        }
        // last steps while we wait for other threads
        std::for_each(it, last, f);

        std::for_each(threads.begin(), threads.end(), [](std::thread& x){x.join();});
    }
    
}

#endif // THREADING_H_ 
