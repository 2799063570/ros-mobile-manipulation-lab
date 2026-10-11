#ifndef AUBO_THESIS_SCENE_OBSTACLES_HPP
#define AUBO_THESIS_SCENE_OBSTACLES_HPP
#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shapes.h>
#include <octomap/OcTree.h>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace aubo_thesis_experiments {
// Newer MoveIt stores local and global shape poses separately. Older ROS1
// versions store world poses directly in shape_poses_. Select at compile time.
template <typename Object>
auto worldShapePose(const Object& object, std::size_t i, int)
    -> decltype(object.global_shape_poses_[i]) { return object.global_shape_poses_[i]; }
template <typename Object>
const Eigen::Isometry3d& worldShapePose(const Object& object, std::size_t i, long) {
  return object.shape_poses_[i];
}
// Conservatively enclose rotated world shapes and occupied octree leaves in
// AABBs. The exact geometry is retained separately for full MoveIt auditing.
inline bool sceneObstacles(const planning_scene::PlanningScene& scene, unsigned int maximum,
                          std::vector<double>& rows, std::string& reason) {
  rows.clear();
  if (maximum == 0 || maximum > 100000) { reason = "max_obstacles must be in [1,100000]"; return false; }
  const auto box = [&](const Eigen::Vector3d& center, const Eigen::Vector3d& size) {
    if (rows.size()/7 >= maximum || !center.allFinite() || !size.allFinite() || (size.array() <= 0).any()) return false;
    rows.insert(rows.end(), {0.0,center.x(),center.y(),center.z(),size.x(),size.y(),size.z()}); return true;
  };
  const auto rotatedBox = [&](const Eigen::Isometry3d& pose, const Eigen::Vector3d& size) {
    return box(pose.translation(), pose.linear().cwiseAbs()*size);
  };
  for (const auto& id : scene.getWorld()->getObjectIds()) {
    const auto object = scene.getWorld()->getObject(id);
    if (object->shapes_.size() != object->shape_poses_.size()) { reason = "shape/pose mismatch"; return false; }
    for (std::size_t i=0; i<object->shapes_.size(); ++i) {
      const auto* shape = object->shapes_[i].get();
      const auto& pose = worldShapePose(*object,i,0);
      bool ok = false;
      if (shape->type == shapes::BOX) {
        const auto* b = static_cast<const shapes::Box*>(shape);
        ok = rotatedBox(pose, Eigen::Vector3d(b->size[0],b->size[1],b->size[2]));
      } else if (shape->type == shapes::SPHERE) {
        const double r = static_cast<const shapes::Sphere*>(shape)->radius;
        ok = rotatedBox(pose, Eigen::Vector3d(2*r,2*r,2*r));
      } else if (shape->type == shapes::CYLINDER) {
        const auto* c = static_cast<const shapes::Cylinder*>(shape);
        ok = rotatedBox(pose, Eigen::Vector3d(2*c->radius,2*c->radius,c->length));
      } else if (shape->type == shapes::MESH) {
        const auto* mesh = static_cast<const shapes::Mesh*>(shape);
        if (mesh->vertex_count == 0) { reason = "empty mesh"; return false; }
        Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()), hi = -lo;
        for (unsigned int v=0; v<mesh->vertex_count; ++v) {
          Eigen::Vector3d p = pose*Eigen::Vector3d(mesh->vertices[3*v],mesh->vertices[3*v+1],mesh->vertices[3*v+2]);
          if (!p.allFinite()) { reason = "nonfinite mesh vertex"; return false; }
          lo = lo.cwiseMin(p); hi = hi.cwiseMax(p);
        }
        ok = box((lo+hi)/2, (hi-lo).cwiseMax(Eigen::Vector3d::Constant(1e-6)));
      } else if (shape->type == shapes::OCTREE) {
        const auto* shape_tree = static_cast<const shapes::OcTree*>(shape);
        if (!shape_tree->octree) { reason = "null octree"; return false; }
        ok = true;
        for (auto it=shape_tree->octree->begin_leafs(); it!=shape_tree->octree->end_leafs(); ++it) {
          if (!shape_tree->octree->isNodeOccupied(*it)) continue;
          const Eigen::Vector3d center = pose*Eigen::Vector3d(it.getX(),it.getY(),it.getZ());
          const Eigen::Vector3d size = pose.linear().cwiseAbs()*Eigen::Vector3d::Constant(it.getSize());
          if (!box(center,size)) { ok = false; break; }
        }
      } else { reason = "unsupported infinite plane/cone/shape; convert explicitly to finite collision geometry"; return false; }
      if (!ok) { reason = "obstacle limit exceeded or invalid shape dimensions; nothing is truncated"; return false; }
    }
  }
  reason = "conservative world AABBs; exact geometry retained in full audit scene"; return true;
}
}  // namespace aubo_thesis_experiments
#endif
