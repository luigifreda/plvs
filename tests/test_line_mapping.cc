#include "Frame.h"
#include "KeyFrame.h"
#include "LineExtractor.h"
#include "LineMatcher.h"
#include "LineProjection.h"
#include "Map.h"
#include "MapLine.h"
#include "GeometricTools.h"
#include "CameraModels/KannalaBrandt8.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace PLVS2;

// Use explicit checks: these tests must also run in Release builds.
static void check(bool condition, const char* message)
{
    if(!condition)
        throw std::runtime_error(message);
}

static cv::line_descriptor_c::KeyLine detection(float x, float y1, float y2, int octave = 0)
{
    cv::line_descriptor_c::KeyLine line;
    line.startPointX = line.endPointX = x;
    line.startPointY = y1;
    line.endPointY = y2;
    line.octave = octave;
    line.angle = std::atan2(y2 - y1, 0.f);
    return line;
}

struct StereoKeyFrame : KeyFrame
{
    StereoKeyFrame(Frame& frame, Map* map, const Sophus::SE3f& rightFromLeft)
        : KeyFrame(frame, map, nullptr)
    {
        mTrl = rightFromLeft;
        mTlr = rightFromLeft.inverse();
    }
};

struct Fixture
{
    Map map;
    KannalaBrandt8 left{std::vector<float>{200, 200, 256, 256, 0, 0, 0, 0}, 1.f};
    KannalaBrandt8 right{std::vector<float>{200, 200, 256, 256, 0, 0, 0, 0}, 1.f};
    KeyFramePtr kf;
    Frame frame;

    explicit Fixture(int octave = 0, bool stereo = true, int featuresPerCamera = 1)
    {
        Frame& f = frame;
        f.mnId = 0;
        f.mTimeStamp = 0;
        f.mnDataset = 0;
        f.N = 0;
        f.Nleft = f.Nright = -1;
        f.NlinesLeft = stereo ? featuresPerCamera : -1;
        f.NlinesRight = stereo ? featuresPerCamera : -1;
        f.Nlines = stereo ? 2 * featuresPerCamera : featuresPerCamera;
        f.mbf = 20;
        f.mbfInv = .05f;
        f.mb = .1f;
        f.mThDepth = 20;
        f.mMedianDepth = 2;
        f.mpORBvocabulary = nullptr;
        f.mpCamera = &left;
        f.mpCamera2 = stereo ? &right : nullptr;
        f.mnScaleLevels = 8; // TUM-VI point pyramid; lines have only 3 levels.
        f.mfScaleFactor = f.mfLineScaleFactor = 1.2f;
        f.mfLogScaleFactor = f.mfLineLogScaleFactor = std::log(1.2f);
        f.mnLineScaleLevels = 3;
        f.mvLineScaleFactors = {1.f, 1.2f, 1.44f};
        f.mvLineLevelSigma2 = {4.f, 5.76f, 8.2944f};
        f.mvLineInvLevelSigma2 = {.25f, 1.f/5.76f, 1.f/8.2944f};
        f.mvKeyLinesUn.assign(featuresPerCamera, detection(256, 246, 266, octave));
        f.mvKeyLines = f.mvKeyLinesUn;
        if(stereo)
        {
            f.mvKeyLinesRightUn = f.mvKeyLinesUn;
            f.mvKeyLinesRight = f.mvKeyLinesUn;
            for(int i = 0; i < featuresPerCamera; ++i)
            {
                f.mvLeftToRightLinesMatch.push_back(i);
                f.mvRightToLeftLinesMatch.push_back(i);
            }
        }
        f.mvuRightLineStart.assign(f.Nlines, 1.f);
        f.mvuRightLineEnd.assign(f.Nlines, 1.f);
        f.mvDepthLineStart.assign(f.Nlines, 2.f);
        f.mvDepthLineEnd.assign(f.Nlines, 2.f);
        f.mLineDescriptors = cv::Mat::zeros(f.Nlines, 32, CV_8U);
        f.mvpMapLines.resize(f.Nlines);
        f.mK_ = left.toLinearK_();
        f.mK = left.toLinearK();
        Frame::fx = Frame::fy = 200;
        Frame::cx = Frame::cy = 256;
        Frame::invfx = Frame::invfy = .005f;
        Frame::mnMinX = Frame::mnMinY = 0;
        Frame::mnMaxX = Frame::mnMaxY = 512;
        Frame::mnMaxDiag = std::sqrt(2.f) * 512;
        Frame::mfGridElementWidthInv = FRAME_GRID_COLS/512.f;
        Frame::mfGridElementHeightInv = FRAME_GRID_ROWS/512.f;
        Frame::mfLineGridElementThetaInv = LINE_THETA_GRID_ROWS/static_cast<float>(LINE_THETA_SPAN);
        Frame::mfLineGridElementDInv = LINE_D_GRID_COLS/(2.f * Frame::mnMaxDiag);
        cv::line_descriptor_c::LSDDetectorC::LSDOptions options;
        options.numOctaves = 3;
        options.scale = 1.2;
        f.mpLineExtractorLeft = std::make_shared<LineExtractor>(10, options);
        f.SetPose(Sophus::SE3f());
        Line2DRepresentation representation;
        Geom2DUtils::GetLine2dRepresentation(256, 246, 256, 266, representation);
        const int col = std::lround((representation.d + Frame::mnMaxDiag) * Frame::mfLineGridElementDInv);
        const int row = std::lround((representation.theta - LINE_THETA_MIN) * Frame::mfLineGridElementThetaInv);
        for(int i = 0; i < featuresPerCamera; ++i)
        {
            f.mLineGrid[col][row].push_back(i);
            if(stereo)
                f.mLineGridRight[col][row].push_back(i);
        }
        kf = KeyFrameNewPtr(f, &map, nullptr);
    }

    MapLinePtr makeLine()
    {
        auto line = MapLineNewPtr(Eigen::Vector3f(0, -.1f, 2), Eigen::Vector3f(0, .1f, 2), kf, &map);
        map.AddMapLine(line);
        return line;
    }

    ~Fixture()
    {
        for(auto& line : map.GetAllMapLines())
            line->SetBadFlag();
    }
};

static void observations()
{
    Fixture f;
    auto line = f.makeLine();
    line->AddObservation(f.kf, 0);
    check(line->Observations() == 2, "left depth observation must count twice");
    check(!line->IsInKeyFrame(f.kf, true), "right observation must initially be missing");
    bool added = false;
    check(AssociateMapLineWithStereo(line, f.kf, 0, &added), "stereo registration failed");
    check(added, "adding the missing stereo partner must report an observation change");
    check(line->Observations() == 3, "stereo registration must count left + right once");
    check(f.kf->GetMapLine(0) == line && f.kf->GetMapLine(1) == line, "both detections must share one map line");
    check(AssociateMapLineWithStereo(line, f.kf, 1, &added), "right-first registration failed");
    check(!added, "unchanged observations must not trigger metadata refresh");
    check(line->Observations() == 3, "repeated registration inflated observation count");
    auto other = f.makeLine();
    check(!AssociateMapLineWithStereo(other, f.kf, 0), "must not overwrite a live association");
    check(other->Observations() == 0, "conflicting registration must not add an observation");
    line->EraseObservation(f.kf);
    check(line->Observations() == 0 && line->isBad(), "removal must balance stereo counts and discard the line");

    Fixture mono(0, false);
    auto monoLine = mono.makeLine();
    check(AssociateMapLineWithStereo(monoLine, mono.kf, 0), "single-image depth registration failed");
    check(monoLine->Observations() == 2, "rectified/RGB-D count changed");

    Fixture fragments(0, true, 2);
    auto repeated = fragments.makeLine();
    AssociateMapLineWithStereo(repeated, fragments.kf, 0);
    fragments.kf->AddMapLine(repeated, 1); // Duplicate same-camera tracking assignment.
    added = true;
    check(!AssociateMapLineWithStereo(repeated, fragments.kf, 1, &added), "same-camera alias must not become a second observation");
    check(!added, "slot cleanup alone must not trigger metadata refresh");
    check(!fragments.kf->GetMapLine(1), "unregistered same-camera alias survived reconciliation");
    check(fragments.kf->GetMapLine(0) == repeated && fragments.kf->GetMapLine(2) == repeated && repeated->Observations() == 3,
          "alias cleanup damaged the registered stereo pair");
}

static void oneSidedMatches()
{
    // Both batch orders and all left/right combinations must recover the free
    // detection without creating another map line or changing existing slots.
    for(bool sourceFirst : {false, true})
    for(int sourceCamera : {0, 1})
    for(int targetCamera : {0, 1})
    {
        Fixture f;
        auto target = KeyFrameNewPtr(f.frame, &f.map, nullptr);
        auto line = f.makeLine();
        AssociateMapLineWithStereo(line, f.kf, sourceCamera);
        line->UpdateNormalAndDepth();
        line->ComputeDistinctiveDescriptors();
        const auto lineCount = f.map.GetAllMapLines().size();
        const bool associated = sourceFirst ? AssociateMatchedMapLine(f.kf, sourceCamera, target, targetCamera)
                                             : AssociateMatchedMapLine(target, targetCamera, f.kf, sourceCamera);
        check(associated, "valid one-sided match was discarded");
        check(target->GetMapLine(0) == line && target->GetMapLine(1) == line, "free stereo pair did not reuse the existing line");
        check(line->Observations() == 6, "one-sided association has incorrect counts");
        check(f.map.GetAllMapLines().size() == lineCount, "one-sided association created a duplicate line");
        check(!AssociateMatchedMapLine(f.kf, sourceCamera, target, targetCamera), "fully associated match must be a no-op");
        check(line->Observations() == 6, "repeated one-sided association inflated counts");
    }

    Fixture f;
    auto target = KeyFrameNewPtr(f.frame, &f.map, nullptr);
    check(!AssociateMatchedMapLine(f.kf, 0, target, 0), "both-free pair must remain available for triangulation");
    auto line = f.makeLine();
    AssociateMapLineWithStereo(line, f.kf, 0);
    // The same image coordinates cannot support this line after a large camera shift.
    target->SetPose(Sophus::SE3f(Eigen::Matrix3f::Identity(), Eigen::Vector3f(2, 0, 0)));
    check(!AssociateMatchedMapLine(f.kf, 0, target, 0), "geometrically inconsistent observation accepted");
    check(!target->GetMapLine(0) && line->Observations() == 3, "rejected observation changed map state");
    target->SetPose(Sophus::SE3f(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0, 0, -3)));
    check(!AssociateMatchedMapLine(f.kf, 0, target, 0), "behind-camera one-sided match accepted");
    target->SetPose(Sophus::SE3f(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0, 0, 3)));
    check(!AssociateMatchedMapLine(f.kf, 0, target, 0), "scale-inconsistent one-sided match accepted");
    f.frame.mvKeyLinesUn = {detection(256, 400, 450)};
    auto disjoint = KeyFrameNewPtr(f.frame, &f.map, nullptr);
    check(!AssociateMatchedMapLine(f.kf, 0, disjoint, 0), "disjoint one-sided match accepted");
    check(!disjoint->GetMapLine(0) && line->Observations() == 3, "invalid one-sided matches changed counts");
    target->SetPose(Sophus::SE3f());
    auto other = f.makeLine();
    AssociateMapLineWithStereo(other, target, 0);
    check(!AssociateMatchedMapLine(f.kf, 0, target, 0), "distinct existing lines must remain for fusion");
    check(target->GetMapLine(0) == other && f.kf->GetMapLine(0) == line, "existing line was overwritten");

    Fixture invalidGeometry;
    auto freeTarget = KeyFrameNewPtr(invalidGeometry.frame, &invalidGeometry.map, nullptr);
    auto overflow = invalidGeometry.makeLine();
    AssociateMapLineWithStereo(overflow, invalidGeometry.kf, 0);
    overflow->SetWorldEndPoints(Eigen::Vector3f(0,-1e28f,1e30f), Eigen::Vector3f(0,1e28f,1e30f));
    check(!AssociateMatchedMapLine(invalidGeometry.kf, 0, freeTarget, 0), "overflowing distance accepted as valid geometry");
    check(!freeTarget->GetMapLine(0), "invalid geometry acquired an observation");

    Fixture fragments(0, true, 2);
    auto occupied = KeyFrameNewPtr(fragments.frame, &fragments.map, nullptr);
    auto mapped = fragments.makeLine();
    AssociateMapLineWithStereo(mapped, fragments.kf, 0);
    AssociateMapLineWithStereo(mapped, occupied, 0);
    check(!AssociateMatchedMapLine(fragments.kf, 0, occupied, 1), "same-camera observation must not be overwritten");
    check(!occupied->GetMapLine(1) && mapped->Observations() == 6, "same-camera conflict mutated the observation set");

    // A right-camera observation must use its own extrinsics and calibration.
    Fixture asymmetric;
    auto existing = asymmetric.makeLine();
    existing->AddObservation(asymmetric.kf, 0);
    asymmetric.kf->AddMapLine(existing, 0);
    asymmetric.right.setLinearParameters({220,210,240,245,0,0,0,0}, 1.f);
    const Sophus::SE3f rightPose(Eigen::AngleAxisf(.1f, Eigen::Vector3f::UnitY()).toRotationMatrix(),
                                Eigen::Vector3f(-.1f,0,0));
    const Eigen::Vector3f start = rightPose * existing->GetWorldPosStart();
    const Eigen::Vector3f end = rightPose * existing->GetWorldPosEnd();
    const Eigen::Vector2f imageStart = asymmetric.right.projectLinear(start);
    const Eigen::Vector2f imageEnd = asymmetric.right.projectLinear(end);
    auto rightDetection = detection(imageStart.x(), imageStart.y(), imageEnd.y());
    rightDetection.endPointX = imageEnd.x();
    asymmetric.frame.mvKeyLinesRightUn = {rightDetection};
    KeyFramePtr stereoTarget(new StereoKeyFrame(asymmetric.frame, &asymmetric.map, rightPose));
    check(AssociateMatchedMapLine(asymmetric.kf, 0, stereoTarget, 1), "valid asymmetric right-camera observation rejected");
    check(stereoTarget->GetMapLine(1) == existing && existing->Observations() == 5,
          "asymmetric stereo association lost an observation");
}

static void replacement()
{
    for(int sourceCamera = 0; sourceCamera < 2; ++sourceCamera)
    {
        Fixture f;
        auto survivor = f.makeLine();
        auto replaced = f.makeLine();
        survivor->AddObservation(f.kf, sourceCamera);
        f.kf->AddMapLine(survivor, sourceCamera);
        replaced->AddObservation(f.kf, 1 - sourceCamera);
        f.kf->AddMapLine(replaced, 1 - sourceCamera);
        replaced->Replace(survivor);
        check(replaced->isBad(), "replacement must retire the duplicate");
        check(survivor->IsInKeyFrame(f.kf, false) && survivor->IsInKeyFrame(f.kf, true), "replacement lost a camera observation");
        check(survivor->Observations() == 3, "replacement count is incorrect");
        check(f.kf->GetMapLine(0) == survivor && f.kf->GetMapLine(1) == survivor, "replacement did not preserve both feature slots");
    }
}

static void distance()
{
    for(int level = 0; level < 3; ++level)
    {
        Fixture f(level);
        auto line = f.makeLine();
        AssociateMapLineWithStereo(line, f.kf, 0);
        line->UpdateNormalAndDepth();
        const float scale = f.kf->mvLineScaleFactors[level];
        check(std::fabs(line->GetMinDistanceInvariance() - .8f*2.f*scale/1.44f) < 1e-5f, "incorrect line scale lower bound");
        check(std::fabs(line->GetMaxDistanceInvariance() - 1.2f*2.f*scale) < 1e-5f, "incorrect midpoint upper bound");
        LineProjection projection;
        FrameTransformsForLineProjection transforms(*f.kf);
        check(projection.ProjectLineWithCheck(line, *f.kf, transforms), "valid reference-view line rejected");
        line->EraseObservation(f.kf);
        auto rightOnly = f.makeLine();
        rightOnly->AddObservation(f.kf, 1);
        f.kf->AddMapLine(rightOnly, 1);
        rightOnly->UpdateNormalAndDepth();
        check(std::fabs(rightOnly->GetMinDistanceInvariance() - .8f*2.f*scale/1.44f) < 1e-5f,
              "right-only reference observation has an invalid scale bound");
    }

    Fixture fixture;
    Frame frame;
    frame.SetPose(Sophus::SE3f());
    frame.mnId = 0;
    frame.NlinesLeft = 1;
    frame.mvKeyLinesUn = {detection(256,246,266)};
    frame.mvKeyLinesRightUn = {detection(256,246,266,2)};
    frame.mvLineScaleFactors = {1.f,1.2f,1.44f};
    frame.mLineDescriptors = cv::Mat::zeros(2,32,CV_8U);
    MapLine temporary(Eigen::Vector3f(0,-.1f,2), Eigen::Vector3f(0,.1f,2), &fixture.map, &frame, 1);
    check(std::fabs(temporary.GetMinDistanceInvariance() - 1.6f) < 1e-5f,
          "temporary right-camera line used the global index in the right-only keyline vector");
}

static void fusion()
{
    Fixture f;
    auto line = f.makeLine();
    line->AddObservation(f.kf, 0);
    f.kf->AddMapLine(line, 0);
    line->UpdateNormalAndDepth();
    line->ComputeDistinctiveDescriptors();
    LineMatcher matcher;
    check(matcher.Fuse(f.kf, std::vector<MapLinePtr>{line}, 3.f, true) == 1, "fusion skipped the missing right camera observation");
    check(line->IsInKeyFrame(f.kf, true) && line->Observations() == 3, "right fusion did not register the observation");
    check(matcher.Fuse(f.kf, std::vector<MapLinePtr>{line}, 3.f, true) == 0, "fusion registered an observation twice");

    Fixture duplicated;
    auto leftLine = duplicated.makeLine();
    auto rightLine = duplicated.makeLine();
    leftLine->AddObservation(duplicated.kf, 0);
    duplicated.kf->AddMapLine(leftLine, 0);
    rightLine->AddObservation(duplicated.kf, 1);
    duplicated.kf->AddMapLine(rightLine, 1);
    leftLine->UpdateNormalAndDepth();
    leftLine->ComputeDistinctiveDescriptors();
    check(matcher.Fuse(duplicated.kf, std::vector<MapLinePtr>{leftLine}, 3.f, true) == 1,
          "fusion failed to merge complementary stereo duplicates");
    check(rightLine->isBad() && duplicated.kf->GetMapLine(1) == leftLine && leftLine->Observations() == 3,
          "fusion lost the right observation while merging duplicates");
}

static void triangulation()
{
    KannalaBrandt8 camera(std::vector<float>{200,200,256,256,0,0,0,0}, 1.f);
    CameraPairTriangulationInput c;
    c.K1 = c.K2 = camera.toLinearK_();
    c.R12.setIdentity();
    c.R21.setIdentity();
    c.t21 = Eigen::Vector3f(-.1f,0,0);
    c.t12 = -c.t21;
    c.H21.setIdentity();
    c.e2 = c.K2*c.t21;
    c.minZ = .1f;
    c.maxZ = 20.f;
    const auto left = detection(256,200,300);
    auto right = detection(246,200,300);
    LineTriangulationOutput out;
    auto triangulate = [&]() { return camera.TriangulateLineMatches(&camera, left, right, 4, 4, c, out); };
    check(triangulate(), "valid stereo pair rejected");
    check(std::fabs(out.depthS - 2.f) < 1e-4f && std::fabs(out.depthE - 2.f) < 1e-4f, "valid stereo depth changed");
    right = detection(246,300,200);
    check(triangulate(), "reversed endpoint ordering rejected");
    right = detection(246,250,350);
    check(triangulate(), "partially overlapping segments rejected");
    right = detection(246,400,500);
    right.endPointX = 250;
    check(!triangulate() && !out.isValid, "disjoint false match accepted");
    right = detection(246,200,200);
    check(!triangulate(), "zero-length segment accepted");
    right = detection(std::numeric_limits<float>::quiet_NaN(),200,300);
    check(!triangulate(), "non-finite detection accepted");
    right = detection(256,200,300);
    check(!triangulate(), "zero-disparity geometry accepted");

    // Exercise the non-rectified path with known geometry and changing poses.
    auto projectedDetection = [&](const Eigen::Vector3f& start, const Eigen::Vector3f& end)
    {
        const Eigen::Vector3f s = c.K2 * (c.R21 * start + c.t21);
        const Eigen::Vector3f e = c.K2 * (c.R21 * end + c.t21);
        auto line = detection(s.x()/s.z(), s.y()/s.z(), e.y()/e.z());
        line.endPointX = e.x()/e.z();
        return line;
    };
    for(int i = -5; i <= 5; ++i)
    {
        c.R21 = Eigen::AngleAxisf(i * .02f, Eigen::Vector3f::UnitY()).toRotationMatrix();
        c.R12 = c.R21.transpose();
        c.H21 = c.K2 * c.R21 * c.K1.inverse();
        right = projectedDetection(Eigen::Vector3f(0,-.56f,2), Eigen::Vector3f(0,.44f,2));
        check(triangulate(), "valid non-rectified pair rejected");
        check(std::fabs(out.depthS - 2.f) < 1e-3f && std::fabs(out.depthE - 2.f) < 1e-3f, "non-rectified depth is incorrect");
    }
    c.R21 = Eigen::AngleAxisf(static_cast<float>(M_PI), Eigen::Vector3f::UnitY()).toRotationMatrix();
    c.R12 = c.R21.transpose();
    c.H21 = c.K2 * c.R21 * c.K1.inverse();
    right = projectedDetection(Eigen::Vector3f(0,-.56f,2), Eigen::Vector3f(0,.44f,2));
    check(!triangulate(), "segment behind the second camera accepted");

    const Eigen::Vector2f p(256,200), q(256,300);
    check(!GeometricTools::CheckLineSegmentProjection(Eigen::Vector3f(0,0,-2), Eigen::Vector3f(0,1,-2), c.K1, p, q, 4), "behind-camera segment accepted");
    check(!GeometricTools::CheckLineSegmentProjection(Eigen::Vector3f(1,-.56f,2), Eigen::Vector3f(1,.44f,2), c.K1, p, q, 4), "large reprojection error accepted");
}

int main(int argc, char** argv)
{
    try
    {
        check(argc == 2, "expected one test name");
        const std::string name(argv[1]);
        if(name == "observations") observations();
        else if(name == "one_sided") oneSidedMatches();
        else if(name == "replacement") replacement();
        else if(name == "distance") distance();
        else if(name == "fusion") fusion();
        else if(name == "triangulation") triangulation();
        else throw std::runtime_error("unknown test: " + name);
        std::cout << "PASS " << name << '\n';
        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
