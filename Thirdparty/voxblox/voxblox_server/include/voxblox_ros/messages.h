#ifndef VOXBLOX_ROS_MESSAGES_H_
#define VOXBLOX_ROS_MESSAGES_H_

#include <vector>

namespace std_msgs
{

struct ColorRGBA
{
    float r;
    float g;
    float b;
    float a;
};
}

namespace voxblox_msgs
{

struct Triangle
{
    // Colored triangle used in meshing

    // Position
    float x[3] = {0, 0, 0};
    float y[3] = {0, 0, 0};
    float z[3] = {0, 0, 0};

    // Color
    uint8_t r[3] = {0, 0, 0};
    uint8_t g[3] = {0, 0, 0};
    uint8_t b[3] = {0, 0, 0};
    uint8_t a[3] = {0, 0, 0};
};

struct MeshBlock
{
    // index of meshed points in block map
    int64_t index[3] = {0, 0, 0};

    std::vector<Triangle> triangles;
};

struct Mesh
{
    std::vector<MeshBlock> mesh_blocks;
};


struct Block
{
  Block()
    : x_index(0)
    , y_index(0)
    , z_index(0)
    , data()  {
    }

   int32_t x_index;
   int32_t y_index;
   int32_t z_index;
   std::vector<uint32_t>  data;

}; 

struct Layer
{
    Layer()
    : voxel_size(0.0)
    , voxels_per_side(0)
    , layer_type()
    , action(0)
    , blocks()
    {
    }

    double voxel_size;
    uint32_t voxels_per_side;
    std::string layer_type;
    uint8_t action;
    std::vector<Block> blocks;

    enum
    {
        ACTION_UPDATE = 0u,
        ACTION_MERGE = 1u,
        ACTION_RESET = 2u,
    };
};  

}

#endif  // VOXBLOX_ROS_CONVERSIONS_H_

