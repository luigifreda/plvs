
#ifndef FAST_FUSION_MANAGER_H
#define FAST_FUSION_MANAGER_H

#include <camerautils/camerautils.hpp>
#include <fusion/geometryfusion_mipmap_cpu.hpp>
#include <fusion/mesh.hpp>

class FastFusionManager 
{

public:
    FastFusionManager(bool createMeshList = true);
    ~FastFusionManager();
    
    std::vector<float> _boundingBox;
    std::vector<std::vector<CameraInfo> > _poses;
    std::vector<std::vector<std::string> > _depthNames;
    std::vector<std::vector<std::string> > _rgbNames;
    
    //  std::vector<Mesh> _meshes;
    MeshSeparate *_currentMeshForSave;
    MeshInterleaved *_currentMeshInterleaved;
    std::vector<PointerMeshDraw*> _pointermeshes;
    
    FusionMipMapCPU* _fusion;
    
    float _imageDepthScale;
    float _maxCamDistance;
    long int _currentFrame;
    long int _currentTrajectory;
    long int _firstFrame;
    long int _nextStopFrame;

    bool _threadFusion;
    boost::thread *_fusionThread;
    bool _newMesh;
    bool _fusionActive;
    bool _fusionAlive;

    bool _threadImageReading;

protected:
    void updateSlot();

protected:
    virtual void init();
    virtual void draw();
    virtual void fastDraw();
    void drawMeshPointer();
    void drawMeshInterleaved();
    void drawCameraFrustum(CameraInfo pose, std::string imageName);
    //virtual void keyPressEvent(QKeyEvent *e);
    void drawGridFrame(float ox, float oy, float oz, float sx, float sy, float sz);
    void setScenePosition(CameraInfo pose);

    void enableLighting();
    void disableLighting();


    long long _lastComputedFrame;

    bool _verbose;
    bool _showCameraFrustum;
    bool _showGridBoundingbox;
    bool _showDepthImage;
    bool _showColor;
    int _displayMode;

    unsigned int _meshNumber;
    unsigned int _fusionNumber;
    float _cx;
    float _cy;
    float _cz;

    std::vector<PointerMeshDraw*> _meshesDraw;
    PointerMeshDraw *_currentMesh;
    unsigned int _currentNV;
    unsigned int _currentNF;
    int _currentMeshType;

    GLuint _vertexBuffer;
    GLuint _faceBuffer;
    GLuint _edgeBuffer;
    GLuint _normalBuffer;
    GLuint _colorBuffer;
    unsigned int _vertexBufferSize;
    unsigned int _faceBufferSize;
    unsigned int _edgeBufferSize;

    bool _onTrack;
    bool _onInterpolation;
    bool _saving;

//    qglviewer::KeyFrameInterpolator _interpolator;
//    std::vector<qglviewer::ManipulatedFrame*> _keyFrames;

    bool _runFusion;
    bool _createMeshList;
    bool _lightingEnabled;
    bool _colorEnabled;

    bool _lightingInternal;

    void generateBuffers();
    bool _buffersGenerated;

};


#endif /* FAST_FUSION_MANAGER_H */
