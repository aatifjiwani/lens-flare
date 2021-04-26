#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <cstdlib>
#include <cmath>
#include <numeric>
#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
//    cout << "start" << endl;

    BBox bbox;

//    cout << "count " << endl;

    // how many elements
    int p_size = std::distance(start, end);
    if (p_size <= 0) {
      return NULL;
    }

//    cout << "bbox " << endl;
    // iterate through all the primitives and add to a bounding box
    for (auto p = start; p != end; p++) {
        BBox bb = (*p)->get_bbox();
        bbox.expand(bb);
    }
    BVHNode *node = new BVHNode(bbox);

    // leaf node
    if (p_size <= max_leaf_size) {
//      cout << "leaf" << endl;
      node->start = start;
      node->end = end;
      node->l = node->r = NULL;
      return node;
    }

    // else the recursion
    // 1. find the splitting point.
    //      -iterate many axes to find it.
    // 2. Assign all elements on one side to the other side of splitting point

    // splitting point construction: choose the mean along some axis.
    // long super_center_x, super_center_y, super_center_z = 0;
//    cout << "retrieve " << endl;
    long double axes_center[3] = {0, 0, 0};
    std::vector <long double> p_locations[3];

    for (auto p = start; p != end; p++) {
        p_locations[0].push_back((*p)->get_bbox().centroid().x);
        p_locations[1].push_back((*p)->get_bbox().centroid().y);
        p_locations[2].push_back((*p)->get_bbox().centroid().z);
    }

    for (int i=0;i<3;i++)
        axes_center[i] = std::accumulate(p_locations[i].begin(), p_locations[i].end(), 0.0)/(0.0 + p_size);

    // The following heuristic may not be very time or space inefficient because
    // we consider many axes.

    // In partitioning, choose the widest axis.
    // x  y  z

    int leftCount[3] = {0, 0, 0};
    int rightCount[3] = {0, 0, 0};

    // determine the counts on each side.
    for (int k=0; k < p_size; k++) {
      // X Y Z iteration
      for (int i=0; i<3; i++) {
        if (p_locations[i][k] < axes_center[i])
          leftCount[i]++;
        else
          rightCount[i]++;
      }
    }

    // determine the best axis to split on, just do this brute force
    // since I don't want to deal with iterators.
    int best_axis = 0;
    for (int i = 1; i < 3; i++) {
      if (std::abs(leftCount[i] - rightCount[i]) < std::abs(leftCount[best_axis] - rightCount[best_axis]))
        best_axis = i;
    }

    // now we have the axes, and we can split along each one.
    // use std::stable_partition and partition_point

//    cout << "partition " << endl;
    std::stable_partition(start, end, [&best_axis, &axes_center](Primitive* p) {
      return p->get_bbox().centroid()[best_axis] < axes_center[best_axis];
    });

    std::vector<Primitive *>::iterator partition_point;
    partition_point = std::partition_point(start, end, [&best_axis, &axes_center](Primitive* p) {
        return p->get_bbox().centroid()[best_axis] < axes_center[best_axis];
    });

    int left_size = std::distance(start, partition_point);
    int right_size = std::distance(partition_point, end);

    if (left_size == 0 || right_size == 0) {
      int new_left_size = (right_size + left_size) / 2;
      partition_point = start+new_left_size;
    }

    left_size = std::distance(start, partition_point);
    right_size = std::distance(partition_point, end);

    node->l = node->r = NULL;
    if (left_size > 0) {
//      cout << "building left " << left_size << endl;
      node->l = construct_bvh(start, partition_point, max_leaf_size);
    }

    if (right_size > 0) {
//      cout << "building right " << right_size << endl;
      node->r = construct_bvh(partition_point, end, max_leaf_size);
    }

//    cout << "done" << endl;
    return node;

}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
  double t0 = ray.min_t;
  double t1 = ray.max_t;
  if (!node->bb.intersect(ray, t0, t1)) {
    return false;
  }

  if (node->isLeaf()) {
      for (auto p = node->start; p != node->end; p++) {
          if((*p)->has_intersection(ray)) {
            return true;
          }
      }
      return false;
  }
  return has_intersection(ray, node->l) || has_intersection(ray, node->r);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // Fill in the intersect function.
  double t0 = ray.min_t;
  double t1 = ray.max_t;
  if (!node->bb.intersect(ray, t0, t1))
      return false;

  if (node->isLeaf()) {
    bool hit = false;
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      hit = (*p)->intersect(ray, i) || hit;
    }
    return hit;
  }


  bool right_intersect = intersect(ray, i, node->l);
  bool left_intersect = intersect(ray, i, node->r);

  return left_intersect || right_intersect;
}

} // namespace SceneObjects
} // namespace CGL
