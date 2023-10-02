/*
 * This file is part of PLVS
 * Copyright (C) 2018-present Luigi Freda <luigifreda at gmail dot com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef EIGEN_UTILS_H
#define EIGEN_UTILS_H

#include <vector>
#include <list>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>


namespace Eigen
{

const char kIoSeparator = ',';

template<class Matrix>
void saveMatrix(const char* filename, const Matrix& matrix, bool binary = false, const char sep = kIoSeparator)
{
    std::ofstream out(filename, std::ios::out | std::ios::binary | std::ios::trunc);
    typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();

    if (binary)
    {
        out.write((char*) (&rows), sizeof (typename Matrix::Index));
        out.write((char*) (&cols), sizeof (typename Matrix::Index));
        out.write((char*) matrix.data(), rows * cols * sizeof (typename Matrix::Scalar));
    }
    else
    {
        for (size_t ii = 0; ii < rows; ii++) 
        {
            for (size_t jj = 0; jj < cols; jj++)            
                out << matrix(ii,jj) << sep;
            out << std::endl;
        }
    }

    out.close();
}

template<class Matrix>
bool loadMatrix(const char* filename, Matrix& matrix, bool binary = false, const char sep = kIoSeparator)
{
    std::ifstream in(filename, std::ios::in | std::ios::binary);

    if (in.fail())
    {
        std::cerr << "ERROR. Cannot find file '" << filename << "'." << std::endl;
        return false;
    }
    
    if (binary)
    {
        typename Matrix::Index rows = 0, cols = 0;        
        in.read((char*) (&rows), sizeof (typename Matrix::Index));
        in.read((char*) (&cols), sizeof (typename Matrix::Index));
        matrix.resize(rows, cols);
        in.read((char *) matrix.data(), rows * cols * sizeof (typename Matrix::Scalar));
    }
    else
    {
        std::string line;
        std::vector<typename Matrix::Scalar> v;
        size_t n_rows = 0;
        while (std::getline(in, line))
        {
            n_rows++;
            std::stringstream lineStream(line);
            std::string cell;

            while (std::getline(lineStream, cell, sep))
            {            
                v.push_back(std::stod(cell));
            }
        }   
   
        size_t n_cols = v.size() / n_rows;
        if ((n_cols * n_rows) != v.size())
        {
            std::cerr << "Error loading mat: " << filename <<", vsize: " << v.size() << std::endl;
        }

        matrix = Matrix(n_rows, n_cols);

        for (int i = 0; i < n_rows; i++)
            for (int j = 0; j < n_cols; j++)
                matrix(i, j) = v[i * n_cols + j];  
    }

    in.close();
    
    return true; 
}

} // namespace Eigen

#endif 
