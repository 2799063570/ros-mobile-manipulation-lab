#include <aubo_sorting_core/occlusion_geometry.hpp>
#include <aubo_sorting_core/footprint.hpp>
#include <gtest/gtest.h>
using namespace aubo_sorting_core;

TEST(OcclusionGeometry, OnlyRobotBetweenCameraAndTargetMasksTheObservation)
{
  const Point3 camera{{0,0,1}},target{{0,0,0}};
  EXPECT_TRUE(robotOccludesTarget(camera,target,{{{-.1,0,.5}},{{.1,0,.5}}},.02));
  EXPECT_TRUE(robotOccludesTarget(camera,target,{{{0,0,.3}}},.02));
  EXPECT_FALSE(robotOccludesTarget(camera,target,{{{.2,0,.5}},{{.3,0,.5}}},.02));
  EXPECT_FALSE(robotOccludesTarget(camera,target,{{{0,0,-.2}},{{0,0,-.3}}},.02));
  EXPECT_FALSE(robotOccludesTarget(camera,target,{{{0,0,1.2}},{{0,0,1.3}}},.02));
  EXPECT_FALSE(robotOccludesTarget(camera,target,{{{0,0,.99}}},.02));
  EXPECT_FALSE(robotOccludesTarget(camera,target,{},.02));
}

TEST(FootprintGeometry, ClipsBothOrientationsAndContainedRectangles)
{
  const Footprint a{{{0,0}},{{2,0}},{{2,2}},{{0,2}}};
  Footprint b{{{1,0}},{{3,0}},{{3,2}},{{1,2}}};
  EXPECT_NEAR(footprintOverlap(a,b),.5,1e-12);
  std::reverse(b.begin(),b.end());
  EXPECT_NEAR(footprintOverlap(a,b),.5,1e-12);
  EXPECT_NEAR(footprintOverlap(a,{{{.5,.5}},{{1,.5}},{{1,1}},{{.5,1}}}),1.,1e-12);
  EXPECT_EQ(footprintOverlap(a,{{{3,0}},{{4,0}},{{4,1}},{{3,1}}}),0.);
  EXPECT_EQ(footprintOverlap(a,{}),0.);
  const Footprint diamond{{{1,0}},{{2,1}},{{1,2}},{{0,1}}};
  EXPECT_NEAR(footprintOverlap(a,diamond),1.,1e-12);
}

TEST(OcclusionGeometry, RecordedRealPregraspPoseCoversBothReportedClassPredictions)
{
  // Read-only TF/queue snapshot at the reported stopped pregrasp pose.
  const Point3 camera{{-.029238,-.652428,.909476}};
  const std::vector<Point3> links{{{-.121030,-.010677,.122000}},{{-.104071,-.202915,.481473}},
      {{-.071523,-.571904,.416946}},{{-.062517,-.674007,.416860}},
      {{-.062578,-.673933,.322860}},{{-.062656,-.673838,.202860}}};
  EXPECT_TRUE(robotOccludesTarget(camera,{{-.062616,-.673908,.034142}},links,.06));
  EXPECT_TRUE(robotOccludesTarget(camera,{{-.063039,-.667543,.034801}},links,.06));
}

TEST(OcclusionGeometry, PartialFootprintOcclusionIsDetectedBeforeCenterIsHidden)
{
  const Point3 camera{{0,0,1}},center{{0,0,0}};
  const std::vector<Point3> outline{{{-.2,-.1,0}},{{.2,-.1,0}},{{.2,.1,0}},{{-.2,.1,0}}};
  const std::vector<Point3> edge_only{{{.1,.05,.5}}};
  EXPECT_FALSE(robotOccludesTarget(camera,center,edge_only,.015));
  EXPECT_TRUE(robotOccludesRegion(camera,center,outline,edge_only,.015));
  const std::vector<Point3> midpoint_only{{{.1,0,.5}}};
  EXPECT_TRUE(robotOccludesRegion(camera,center,outline,midpoint_only,.015));
  EXPECT_FALSE(robotOccludesRegion(camera,center,outline,{{{.4,0,.5}}},.015));
  EXPECT_FALSE(robotOccludesRegion(camera,center,outline,{{{.1,.05,-.5}}},.015));
  EXPECT_FALSE(robotOccludesRegion(camera,center,{},edge_only,.015));
}

TEST(OcclusionGeometry, CurrentStoppedPoseMasksTheReportedBottleTarget)
{
  // Read-only snapshot: records/occlusion_fix_20261110_094856/stopped_pose.json.
  const Point3 camera{{-.0292379638,-.6524278933,.9094763530}};
  const Point3 target{{-.1405319272,-.6207131473,.0539611569}};
  const std::vector<Point3> links{{{.1112017953,-.0489531482,.122}},
      {{.0485615476,-.1912450922,.4992177117}},{{-.0992566755,-.5270183348,.4168641902}},
      {{-.1405549045,-.6208304043,.4168205641}},{{-.1405524492,-.6207877716,.3228205738}},
      {{-.1405491371,-.6207329434,.2028205864}}};
  EXPECT_TRUE(robotOccludesRegion(camera,target,{},links,.06));
}

int main(int argc,char** argv) {testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
