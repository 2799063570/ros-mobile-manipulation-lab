#include <aubo_sorting_core/instance_queue.hpp>
#include <iostream>
#include <limits>
#include <stdexcept>

#define CHECK(condition) do { if (!(condition)) throw std::runtime_error(#condition); } while (false)
using Queue = aubo_sorting_core::InstanceQueue<int>;
Queue::Sample object(double x, std::string color = "red", int payload = 7)
{
  Queue::Sample sample;
  sample.color = color; sample.x = x; sample.z = .12; sample.payload = payload;
  return sample;
}
void confirm(Queue& queue, const std::vector<Queue::Sample>& samples, double start = 1)
{
  for (int i = 0; i < 5; ++i) queue.update(samples, start + .1*i);
}
int main()
{
  try {
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
      CHECK(q.reserve({"red"},1.5,t)); q.update({object(.53)},1.6);
      CHECK(!q.executionValid(t.id,1.6)); CHECK(q.tracks().at(t.id).sample.x==.5);
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
    std::cout << "12 instance queue scenarios passed\n";
    return 0;
  } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
