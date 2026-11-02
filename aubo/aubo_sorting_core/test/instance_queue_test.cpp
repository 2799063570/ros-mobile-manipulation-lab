#include <aubo_sorting_core/instance_queue.hpp>
#include <gtest/gtest.h>
#include <limits>

#define CHECK(condition) ASSERT_TRUE(condition)
using Queue = aubo_sorting_core::InstanceQueue<int>;
Queue::Sample object(double x, std::string category = "red", int payload = 7)
{
  Queue::Sample sample;
  sample.category = category; sample.x = x; sample.z = .12; sample.payload = payload;
  return sample;
}
void confirm(Queue& queue, const std::vector<Queue::Sample>& samples, double start = 1)
{
  for (int i = 0; i < 5; ++i) queue.update(samples, start + .1*i);
}
TEST(InstanceQueue, MaintainsTrackingAndReservationInvariants)
{
    { // Multiple same-class targets survive input reordering; reservations are unique.
      Queue q;
      for (int i=0; i<5; ++i)
        q.update(i%2 ? std::vector<Queue::Sample>{object(.56),object(.50)} :
                       std::vector<Queue::Sample>{object(.50),object(.56)}, 1+.1*i);
      CHECK(q.tracks().size() == 2);
      Queue::Track a,b,c;
      CHECK(q.reserve({"red"},1.5,a)); CHECK(q.reserve({"red"},1.5,b));
      CHECK(a.id != b.id); CHECK(!q.reserve({"red"},1.5,c));
      CHECK(std::abs(a.sample.x-b.sample.x) > .05);
    }
    { // Duplicate boxes cannot add observations twice in a single frame.
      Queue q; q.update({object(.5),object(.502)},1);
      CHECK(q.tracks().size()==1); CHECK(q.tracks().begin()->second.observations==1);
      Queue::Track t; CHECK(!q.reserve({"red"},1,t));
    }
    { // Background updates/additions do not change the execution snapshot.
      Queue q; confirm(q,{object(.5)}); Queue::Track t,next;
      CHECK(q.reserve({"red"},1.5,t));
      confirm(q,{object(.502,"red",99),object(.8,"blue")},1.6);
      CHECK(t.sample.payload==7); CHECK(q.tracks().at(t.id).sample.payload==7);
      CHECK(q.reserve({"blue"},2.1,next)); CHECK(next.id!=t.id);
      CHECK(!q.reserve({"red"},2.1,next));
    }
    { // A nearby moved reserved target cancels descent, without changing its pose.
      Queue q; confirm(q,{object(.5)}); Queue::Track t;
      CHECK(q.reserve({"red"},1.5,t)); q.update({object(.538)},1.6);
      CHECK(!q.executionValid(t.id,1.6)); CHECK(q.tracks().at(t.id).sample.x==.5);
    }
    { // 手眼相机随臂移动引起的约 3 cm 轮廓中心偏差不应取消预留。
      Queue q; confirm(q,{object(.5)}); Queue::Track t;
      CHECK(q.reserve({"red"},1.5,t)); q.update({object(.53)},1.6);
      CHECK(q.executionValid(t.id,1.6));
      CHECK(q.tracks().at(t.id).sample.x==.5);
    }
    { // 目标跳出关联半径后，不能沿用旧预留坐标继续下降。
      Queue q; confirm(q,{object(.5)}); Queue::Track t;
      CHECK(q.reserve({"red"},1.5,t)); q.update({object(.56)},1.6);
      CHECK(!q.executionValid(t.id,1.6));
    }
    { // 预抓取运动遮挡目标时可延长预留期；已观察到的位移仍立即否决。
      Queue q; confirm(q,{object(.5)}); Queue::Track t;
      CHECK(q.reserve({"red"},1.5,t));
      q.update({},12);
      CHECK(!q.executionValid(t.id,12));
      CHECK(q.executionValid(t.id,12,30));
      q.update({object(.538)},13);
      CHECK(!q.executionValid(t.id,13,30));
      CHECK(!q.executionValid(t.id,35,30));
    }
    { // A neighbouring same-class target inside the association radius is not consumed by the reservation.
      Queue q; confirm(q,{object(.5),object(.53)}); Queue::Track a,b;
      CHECK(q.reserve({"red"},1.5,a)); q.update({object(.53),object(.5)},1.6);
      CHECK(q.executionValid(a.id,1.6)); CHECK(q.reserve({"red"},1.6,b)); CHECK(a.id!=b.id);
    }
    { // Brief occlusion retains a target; long occlusion prevents dispatch.
      Queue q; confirm(q,{object(.5)}); q.update({},2); Queue::Track t;
      CHECK(q.reserve({"red"},2,t));
      Queue old; confirm(old,{object(.5)}); CHECK(!old.reserve({"red"},6,t));
      old.update({},40); CHECK(old.tracks().empty());
    }
    { // A positional jump must not leave a ready ghost at the previous position.
      Queue q; confirm(q,{object(.5)}); q.update({object(.7)},1.6); Queue::Track t;
      CHECK(!q.reserve({"red"},1.7,t)); confirm(q,{object(.7)},1.8);
      CHECK(q.reserve({"red"},2.3,t)); CHECK(t.sample.x==.7);
    }
    { // Release quarantine rejects residual frames, but a new object at that spot can re-confirm.
      Queue q; confirm(q,{object(.5)}); Queue::Track t,next;
      CHECK(q.reserve({"red"},1.5,t)); q.finish(t.id,true,2);
      confirm(q,{object(.5)},2.1); CHECK(!q.reserve({"red"},2.6,next));
      q.update({object(.5)},4.1); CHECK(!q.reserve({"red"},4.1,next));
      confirm(q,{object(.5)},4.2); CHECK(q.reserve({"red"},4.7,next));
    }
    { // Failed execution is not DONE and invalidates its neighbours.
      Queue q; confirm(q,{object(.5),object(.8)}); Queue::Track t,next;
      CHECK(q.reserve({"red"},1.5,t)); q.finish(t.id,false,1.6);
      CHECK(q.tracks().at(t.id).status==Queue::Status::REVIEW);
      CHECK(!q.reserve({"red"},1.7,next));
    }
    { // Invalid geometry and non-finite coordinates never become executable.
      Queue q; auto bad=object(.5); bad.eligible=false; confirm(q,{bad});
      Queue::Track t; CHECK(!q.reserve({"red"},1.5,t));
      q.update({object(.5)},1.6); CHECK(!q.reserve({"red"},1.6,t));
      bad.x=std::numeric_limits<double>::quiet_NaN(); q.update({bad},2);
      CHECK(q.tracks().size()==1);
    }
    { // Workspace reset drops tracks and does not reuse an externally visible ID.
      Queue q; confirm(q,{object(.5)}); Queue::Track a,b;
      CHECK(q.reserve({"red"},1.5,a)); q.clear(); confirm(q,{object(.5)},2);
      CHECK(q.reserve({"red"},2.5,b)); CHECK(b.id>a.id);
    }
    { // Capacity and observation-gap bounds prevent unbounded growth/old confirmation reuse.
      Queue q; q.capacity=2; q.update({object(.1),object(.3),object(.5)},1);
      CHECK(q.tracks().size()==2); confirm(q,{object(.1)},2);
      q.update({object(.1)},4); Queue::Track t; CHECK(!q.reserve({"red"},4,t));
    }
}

TEST(InstanceQueue, DispatchesAcrossCategoriesInArrivalOrder)
{
  for (const auto& allowed : {std::vector<std::string>{"bottle", "can", "box"},
                             std::vector<std::string>{"box", "can", "bottle"}})
  {
    Queue q;
    confirm(q, {object(.1, "can", 1), object(.3, "box", 2),
                object(.5, "bottle", 3), object(.7, "can", 4)});
    Queue::Track target;
    for (int expected = 1; expected <= 4; ++expected)
    {
      ASSERT_TRUE(q.reserve(allowed, 1.5, target));
      EXPECT_EQ(target.sample.payload, expected);
    }
    EXPECT_FALSE(q.reserve(allowed, 1.5, target));
  }
}

TEST(InstanceQueue, MissingCategoriesDoNotBlockAndUnknownCategoriesAreExcluded)
{
  Queue q;
  confirm(q, {object(.1, "unknown"), object(.3, "can", 1), object(.5, "can", 2)});
  Queue::Track target;
  EXPECT_FALSE(q.reserve({}, 1.5, target));
  for (int expected = 1; expected <= 2; ++expected)
  {
    ASSERT_TRUE(q.reserve({"bottle", "can", "box"}, 1.5, target));
    EXPECT_EQ(target.sample.category, "can");
    EXPECT_EQ(target.sample.payload, expected);
  }
  EXPECT_FALSE(q.reserve({"bottle", "can", "box"}, 1.5, target));
}

TEST(InstanceQueue, OlderUnconfirmedAndStaleTargetsDoNotBlockReadyInstances)
{
  Queue q;
  q.update({object(.1, "can")}, 1.);
  confirm(q, {object(.3, "box")}, 1.1);
  Queue::Track target;
  ASSERT_TRUE(q.reserve({"can", "box"}, 1.6, target));
  EXPECT_EQ(target.sample.category, "box");

  Queue stale;
  confirm(stale, {object(.1, "can")});
  confirm(stale, {object(.3, "box")}, 5.);
  ASSERT_TRUE(stale.reserve({"can", "box"}, 5.5, target));
  EXPECT_EQ(target.sample.category, "box");
}

Queue::Sample rectangle(double x,std::string category,double confidence,int payload=7,double width=.06)
{
  auto sample=object(x,category,payload);
  sample.confidence=confidence;
  sample.footprint={{{x-width/2,-.02}},{{x+width/2,-.02}},{{x+width/2,.02}},{{x-width/2,.02}}};
  return sample;
}

TEST(InstanceQueue, MergesOverlappingClassesByConfidenceWithoutDoubleCountingFrames)
{
  for (bool reverse:{false,true}) {
    Queue q;
    auto can=rectangle(.5,"can",.9,90), box=rectangle(.506,"box",.7,70);
    confirm(q,reverse ? std::vector<Queue::Sample>{box,can}:std::vector<Queue::Sample>{can,box});
    ASSERT_EQ(q.tracks().size(),1u);
    const auto& track=q.tracks().begin()->second;
    EXPECT_EQ(track.observations,5u);
    EXPECT_EQ(track.sample.category,"can");
    EXPECT_EQ(track.sample.payload,90);
    EXPECT_DOUBLE_EQ(track.sample.x,.5);
    EXPECT_DOUBLE_EQ(track.sample.confidence,.9);
    EXPECT_EQ(q.mergedDetectionsTotal(),5u);
  }
}

TEST(InstanceQueue, RequiresBothProximityAndAreaOverlap)
{
  Queue near;
  confirm(near,{rectangle(.5,"can",.9,1,.004),rectangle(.506,"box",.8,2,.004)});
  EXPECT_EQ(near.tracks().size(),2u);
  Queue far;
  auto a=rectangle(.5,"can",.9), b=rectangle(.53,"box",.8);
  b.footprint=a.footprint;
  confirm(far,{a,b});
  EXPECT_EQ(far.tracks().size(),2u);
  Queue unknown;
  confirm(unknown,{object(.5,"can"),object(.506,"box")});
  EXPECT_EQ(unknown.tracks().size(),2u);
  Queue touching;
  confirm(touching,{rectangle(.5,"can",.9,1,.004),rectangle(.506,"can",.8,2,.004)});
  EXPECT_EQ(touching.tracks().size(),2u);
  Queue stacked;
  auto low=rectangle(.5,"can",.9), high=rectangle(.5,"box",.8);
  high.z+=.1;
  confirm(stacked,{low,high});
  EXPECT_EQ(stacked.tracks().size(),2u);
}

TEST(InstanceQueue, ConfidenceSelectionPreservesGroupArrivalOrder)
{
  Queue q;
  confirm(q,{rectangle(.5,"can",.6,1),rectangle(.8,"bottle",.99,2),rectangle(.506,"box",.9,3)});
  Queue::Track first,second;
  ASSERT_TRUE(q.reserve({"bottle","can","box"},1.5,first));
  ASSERT_TRUE(q.reserve({"bottle","can","box"},1.5,second));
  EXPECT_EQ(first.sample.payload,3);
  EXPECT_EQ(second.sample.payload,2);
}

TEST(InstanceQueue, ClassFlickerKeepsIdentityAndBetterClassRequiresReconfirmation)
{
  Queue q;
  confirm(q,{rectangle(.5,"can",.9,90)});
  const auto id=q.tracks().begin()->first;
  q.update({rectangle(.506,"box",.7,70)},1.5);
  ASSERT_EQ(q.tracks().size(),1u);
  EXPECT_EQ(q.tracks().begin()->first,id);
  EXPECT_EQ(q.tracks().at(id).sample.category,"can");
  q.update({rectangle(.506,"box",.95,95)},1.6);
  EXPECT_EQ(q.tracks().at(id).sample.category,"box");
  EXPECT_EQ(q.tracks().at(id).observations,1u);
  Queue::Track reserved;
  EXPECT_FALSE(q.reserve({"box"},1.6,reserved));
  confirm(q,{rectangle(.506,"box",.95,95)},1.7);
  EXPECT_TRUE(q.reserve({"box"},2.2,reserved));
  EXPECT_EQ(reserved.id,id);
}

TEST(InstanceQueue, HistoricalDuplicatesMergeWithoutSummingConfirmationEvidence)
{
  Queue q;
  confirm(q,{rectangle(.5,"can",.8),rectangle(.53,"box",.9)});
  ASSERT_EQ(q.tracks().size(),2u);
  const auto first_id=q.tracks().begin()->first;
  q.update({rectangle(.5,"can",.8),rectangle(.506,"box",.95)},1.5);
  ASSERT_EQ(q.tracks().size(),1u);
  EXPECT_EQ(q.tracks().begin()->first,first_id);
  EXPECT_EQ(q.tracks().begin()->second.sample.category,"box");
  EXPECT_LT(q.tracks().begin()->second.observations,q.min_observations);
}

TEST(InstanceQueue, ReservedSnapshotSurvivesDuplicateClassButNotActualMovement)
{
  Queue q;
  confirm(q,{rectangle(.5,"can",.85,85)});
  Queue::Track reserved;
  ASSERT_TRUE(q.reserve({"can"},1.5,reserved));
  q.update({rectangle(.506,"box",.99,99)},1.6);
  EXPECT_TRUE(q.executionValid(reserved.id,1.6));
  EXPECT_EQ(q.tracks().at(reserved.id).sample.category,"can");
  EXPECT_EQ(q.tracks().at(reserved.id).sample.payload,85);
  EXPECT_EQ(q.tracks().size(),1u);
  q.update({rectangle(.538,"can",.99,99)},1.7);
  EXPECT_FALSE(q.executionValid(reserved.id,1.7));
}

TEST(InstanceQueue, UnrelatedClassCannotRefreshReservedTarget)
{
  Queue q;
  confirm(q,{rectangle(.5,"can",.85,85,.004)});
  Queue::Track reserved;
  ASSERT_TRUE(q.reserve({"can"},1.5,reserved));
  q.update({rectangle(.516,"box",.99,99,.004)},1.6);
  EXPECT_FALSE(q.executionValid(reserved.id,1.6));
}

TEST(InstanceQueue, DuplicateResidualCannotBypassDoneHold)
{
  Queue q; confirm(q,{rectangle(.5,"can",.85)});
  Queue::Track reserved;
  ASSERT_TRUE(q.reserve({"can"},1.5,reserved));
  q.finish(reserved.id,true,1.6);
  confirm(q,{rectangle(.506,"box",.99)},1.7);
  EXPECT_EQ(q.tracks().size(),1u);
  EXPECT_FALSE(q.reserve({"box","can"},2.2,reserved));
}

TEST(InstanceQueue, MergingCannotEraseAQuarantineFaultFromTheActiveReservation)
{
  Queue q;confirm(q,{rectangle(.5,"can",.85),rectangle(.53,"box",.9)});
  Queue::Track active,other;
  ASSERT_TRUE(q.reserve({"can"},1.5,active));
  ASSERT_TRUE(q.reserve({"box"},1.5,other));
  q.finish(other.id,false,1.6);
  q.update({rectangle(.5,"can",.85),rectangle(.506,"box",.99)},1.7);
  EXPECT_EQ(q.tracks().size(),1u);
  EXPECT_FALSE(q.executionValid(active.id,1.7));
  EXPECT_EQ(q.tracks().at(active.id).sample.category,"can");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
