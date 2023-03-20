#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

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

    // TODO (Part 2.1):
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code build a BVH aggregate with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.
    
    
    BBox bbox;
    int num_prims = 0;
    for (auto p = start; p != end; p++) {
      BBox bb = (*p)->get_bbox();
      bbox.expand(bb);
      num_prims++;
    }
    
    BVHNode* node = new BVHNode(bbox);
    //leaf
    if (num_prims <= max_leaf_size) {
        node->start = start;
        node->end = end;
        node->l = NULL;
        node->r = NULL;
    }
    //inner
    else {
        
        int split = 0;
        double best_cost = INFINITY;
        Vector3D best_centroid;
        double prev_cost = -INFINITY;
        for (int i = 0; i < 3; i++) {
            Vector3D centr;
            //loop over all primitive centroids, calculate cost, find best
            for (auto p = start; p < end; p++) {
                Vector3D centr = (*p)->get_bbox().centroid();
                BBox left_box, right_box;
                int left_size = 0, right_size = 0;
                for (auto p = start; p < end; p++) {
                    if ((*p)->get_bbox().centroid()[i] <= centr[i]) {
                        left_box.expand((*p)->get_bbox());
                        left_size++;
                    }
                    else {
                        right_box.expand((*p)->get_bbox());
                        right_size++;
                    }
                }
                double cost = (left_size)*left_box.surface_area() + (right_size)*right_box.surface_area();
    
                if (cost < best_cost) {
                    best_cost = cost;
                    split = i;
                    best_centroid = centr;
                }   
            }
            
        }

        vector<Primitive*> *left_bckt = new vector<Primitive*>(); 
        vector<Primitive*>* right_bckt = new vector<Primitive*>();

   /*     std::sort(start, end, [&](Primitive* a, Primitive* b) {
            return a->get_bbox().centroid()[split] < b->get_bbox().centroid()[split];
            });*/

        for (auto p = start; p < end; p++) {
            if ((*p)->get_bbox().centroid()[split] <= best_centroid[split]) {
                left_bckt->push_back(*p);
            }
            else {
                right_bckt->push_back(*p);
            }
        }
        if (left_bckt->empty() || right_bckt->empty()) {
            /*node->start = start;
            node->end = end;
            return node;*/
            //means they are all on the exact center of split axis:
            best_cost = INFINITY;
            int new_split = 0;
            for (int i = 0; i < 3; i++) {
                if (i == split) {
                    continue;
                }
                BBox left_box, right_box;
                int left_size = 0, right_size = 0;
                for (auto p = start; p < end; p++) {
                    if ((*p)->get_bbox().centroid()[i] < best_centroid[i]) {
                        left_box.expand((*p)->get_bbox());
                        left_size++;
                    }
                    else {
                        right_box.expand((*p)->get_bbox());
                        right_size++;
                    }
                }
                double cost = (left_size)*left_box.surface_area() + (right_size)*right_box.surface_area();
                if (cost < best_cost) {
                    best_cost = cost;
                    new_split = i;
                }              

            }
            left_bckt->clear();
            right_bckt->clear();

            for (auto p = start; p < end; p++) {
                if ((*p)->get_bbox().centroid()[split] < best_centroid[split]) {
                    left_bckt->push_back(*p);
                }
                else {
                    right_bckt->push_back(*p);
                }
            }
            
            //int bckt_size = (left_bckt->size() + right_bckt->size());
            //int m_ind = bckt_size / 2;
            ////partitions merged and ordered by split axis
            //std::vector<Primitive*>::iterator merged_start, merged_end;
            //merged_end = std::merge(left_bckt->begin(), left_bckt->end(),
            //    right_bckt->begin(), right_bckt->end(), merged_start, 
            //    [&](const auto& lhs, const auto& rhs) {
            //        return lhs->get_bbox().centroid()[split] < rhs->get_bbox().centroid()[split];
            //    });
            ////arrange
            //auto p = merged_start;
            //left_bckt->clear();
            //right_bckt->clear();
            //for (int i = 0; i < bckt_size; i++, p++) {
            //    if (i < m_ind) {
            //        left_bckt->push_back(*p);
            //    }
            //    else {
            //        right_bckt->push_back(*p);
            //    }
            //}
        }

        node->l = construct_bvh((*left_bckt).begin(), (*left_bckt).end(), max_leaf_size);
        node->r = construct_bvh((*right_bckt).begin(), (*right_bckt).end(), max_leaf_size);




    }
    return node;



}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
      // TODO (Part 2.3):
      // Fill in the intersect function.
      // Take note that this function has a short-circuit that the
      // Intersection version cannot, since it returns as soon as it finds
      // a hit, it doesn't actually have to find the closest hit.
   
    BBox node_bbox = node->bb;
    if (!node_bbox.intersect(ray, ray.min_t, ray.max_t)) {
        return false;
    }
    if (node->isLeaf()) {
        for (auto p = node->start; p < node->end; p++) {
            if ((*p)->has_intersection(ray)) {
                return true;
            }
        }
        return false;
    }
    if (has_intersection(ray, node->l)) {
        return true;
    }
    else {
        return has_intersection(ray, node->r);
    }
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
    BBox node_bbox = node->bb;
    if (!node_bbox.intersect(ray, ray.min_t, ray.max_t)) {
        return false;
    }
    total_isects++;
    if (node->isLeaf()) {
        bool intersected = false;
        for (auto p = node->start; p < node->end; p++) {
            total_isects++;
            intersected = (*p)->intersect(ray, i) || intersected;
        }
        return intersected;
    }
    bool hit1 = intersect(ray, i, node->l);
    bool hit2 = intersect(ray, i, node->r);
    return hit1 || hit2;



 /* bool hit = false;
  for (auto p : primitives) {
    total_isects++;
    hit = p->intersect(ray, i) || hit;
  }
  return hit;*/


}

} // namespace SceneObjects
} // namespace CGL
