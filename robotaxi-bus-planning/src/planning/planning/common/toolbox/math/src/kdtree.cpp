

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <vector>
#include <iostream>

#include "common/toolbox/math/include/kdtree.h"

namespace toolbox {
namespace math    {
namespace kdtree  {
/*
 * @brief  some useful functions
 * @param  none
 * @return none
 */
KDNodePtr NewKDNodePtr() {
  KDNodePtr mynode = std::make_shared<KDNode>();
  return mynode;
}

double dist2(const point_t &a, const point_t &b) {
  double distc = 0;
  for (size_t i = 0; i < a.size(); i++) {
    double di = a.at(i) - b.at(i);
    distc += di * di;
  }
  return distc;
}

double dist2(const KDNodePtr &a, const KDNodePtr &b) { return dist2(a->x, b->x); }

void sort_on_idx(const pointIndexArr::iterator &begin,  //
                 const pointIndexArr::iterator &end,    //
                 size_t dim) {
  comparer comp(dim);
  comp.dim = dim;

  using std::placeholders::_1;
  using std::placeholders::_2;

  std::sort(begin, end, std::bind(&comparer::compare_dim, comp, _1, _2));
}

KDTree::KDTree(pointVec point_array) {
  leaf = std::make_shared<KDNode>();
  // iterators
  pointIndexArr arr;
  for (size_t i = 0; i < point_array.size(); i++) arr.push_back(pointIndex(point_array.at(i), i));

  auto begin = arr.begin();
  auto end   = arr.end();

  size_t length = arr.size();
  size_t level  = 0;  // starting

  p_num = length;

  root = KDTree::make_tree(begin, end, length, level);
}

KDNodePtr KDTree::make_tree(const pointIndexArr::iterator &begin,  //
                            const pointIndexArr::iterator &end,    //
                            const size_t &length,                  //
                            const size_t &level                    //
) {
    if (begin == end) return NewKDNodePtr();  // empty tree

    size_t dim = begin->first.size();

    if (length > 1) sort_on_idx(begin, end, level);

    auto middle  = begin + (size_t)(length / 2);

    auto l_begin = begin;
    auto l_end   = middle;
    auto r_begin = middle + 1;
    auto r_end   = end;

    size_t l_len = length / 2;
    size_t r_len = length - l_len - 1;

    KDNodePtr left;
    if (l_len > 0 && dim > 0) {
      left = make_tree(l_begin, l_end, l_len, (level + 1) % dim);
    } else {
      left = leaf;
    }
    KDNodePtr right;
    if (r_len > 0 && dim > 0) {
      right = make_tree(r_begin, r_end, r_len, (level + 1) % dim);
    } else {
      right = leaf;
    }

    // KDNode result = KDNode();
    return std::make_shared<KDNode>(*middle, left, right);
}

void KDTree::insert_point (KDNodePtr &root, point_t &to_insert, size_t level) {
  // check root
  if (nullptr == root) {
    leaf = std::make_shared<KDNode>();

    p_num       = 1;
    root        = std::make_shared<KDNode>();
    root->index = 0;
    root->x     = to_insert;
    root->left  = leaf;
    root->right = leaf; 
    return;
  } 
  // normal process
  size_t dim     = root->x.size();
  size_t cmp_dim = level % dim;
  
  if (to_insert.at(cmp_dim) <= root->x.at(cmp_dim)) {
    if (!(*root->left)) {
      auto new_node   = NewKDNodePtr();
      new_node->index = p_num++;
      new_node->x     = to_insert;
      new_node->left  = leaf;
      new_node->right = leaf;
      root->left      = new_node;
      return;
    } else {
      insert_point(root->left, to_insert, ++level);
    }
  } else {
    if (!(*root->right)) {
      auto new_node   = NewKDNodePtr();
      new_node->index = p_num++;
      new_node->x     = to_insert;
      new_node->left  = leaf;
      new_node->right = leaf;
      root->right     = new_node;
      return;
    } else {
      insert_point(root->right, to_insert, ++level);
    }
  }
}

KDNodePtr KDTree::nearest_(   //
    const KDNodePtr &branch,  //
    const point_t &pt,        //
    const size_t &level,      //
    const KDNodePtr &best,    //
    const double &best_dist   //
) {
    double d, dx, dx2;

    if (!bool(*branch)) {
        return NewKDNodePtr();  // basically, null
    }

    point_t branch_pt(*branch);
    size_t dim = branch_pt.size();

    d = dist2(branch_pt, pt);
    dx = branch_pt.at(level) - pt.at(level);
    dx2 = dx * dx;

    KDNodePtr best_l = best;
    double best_dist_l = best_dist;

    if (d < best_dist) {
        best_dist_l = d;
        best_l = branch;
    }

    size_t next_lv = (level + 1) % dim;
    KDNodePtr section;
    KDNodePtr other;

    // select which branch makes sense to check
    if (dx > 0) {
        section = branch->left;
        other = branch->right;
    } else {
        section = branch->right;
        other = branch->left;
    }

    // keep nearest neighbor from further down the tree
    KDNodePtr further = nearest_(section, pt, next_lv, best_l, best_dist_l);
    if (!further->x.empty()) {
        double dl = dist2(further->x, pt);
        if (dl < best_dist_l) {
            best_dist_l = dl;
            best_l = further;
        }
        // only check the other branch if it makes sense to do so
        if (dx2 < best_dist_l) {
            further = nearest_(other, pt, next_lv, best_l, best_dist_l);
            if (!further->x.empty()) {
                dl = dist2(further->x, pt);
                if (dl < best_dist_l) {
                    best_dist_l = dl;
                    best_l = further;
                }
            }
        }
    }

    return best_l;
};

// default caller
KDNodePtr KDTree::nearest_(const point_t &pt) {
    size_t level = 0;
    // KDNodePtr best = branch;
    double branch_dist = dist2(point_t(*root), pt);
    return nearest_(root,          // beginning of tree
                    pt,            // point we are querying
                    level,         // start from level 0
                    root,          // best is the root
                    branch_dist);  // best_dist = branch_dist
};

point_t KDTree::nearest_point(const point_t &pt) {
    return point_t(*nearest_(pt));
};
size_t KDTree::nearest_index(const point_t &pt) {
    return size_t(*nearest_(pt));
};

pointIndex KDTree::nearest_pointIndex(const point_t &pt) {
    KDNodePtr Nearest = nearest_(pt);
    return pointIndex(point_t(*Nearest), size_t(*Nearest));
}

pointIndexArr KDTree::neighborhood_(  //
    const KDNodePtr &branch,          //
    const point_t &pt,                //
    const double &rad,                //
    const size_t &level               //
) {
    double d, dx, dx2;

    if (!bool(*branch)) {
        // branch has no point, means it is a leaf,
        // no points to add
        return pointIndexArr();
    }

    size_t dim = pt.size();

    double r2 = rad * rad;

    d = dist2(point_t(*branch), pt);
    dx = point_t(*branch).at(level) - pt.at(level);
    dx2 = dx * dx;

    pointIndexArr nbh, nbh_s, nbh_o;
    if (d <= r2) {
        nbh.push_back(pointIndex(*branch));
    }

    //
    KDNodePtr section;
    KDNodePtr other;
    if (dx > 0) {
        section = branch->left;
        other = branch->right;
    } else {
        section = branch->right;
        other = branch->left;
    }

    nbh_s = neighborhood_(section, pt, rad, (level + 1) % dim);
    nbh.insert(nbh.end(), nbh_s.begin(), nbh_s.end());
    if (dx2 < r2) {
        nbh_o = neighborhood_(other, pt, rad, (level + 1) % dim);
        nbh.insert(nbh.end(), nbh_o.begin(), nbh_o.end());
    }

    return nbh;
};

pointIndexArr KDTree::neighborhood(  //
    const point_t &pt,               //
    const double &rad) {
    size_t level = 0;
    return neighborhood_(root, pt, rad, level);
}

pointVec KDTree::neighborhood_points(  //
    const point_t &pt,                 //
    const double &rad) {
    size_t level = 0;
    pointIndexArr nbh = neighborhood_(root, pt, rad, level);
    pointVec nbhp;
    nbhp.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), nbhp.begin(),
                   [](pointIndex x) { return x.first; });
    return nbhp;
}

indexArr KDTree::neighborhood_indices(  //
    const point_t &pt,                  //
    const double &rad) {
    size_t level = 0;
    pointIndexArr nbh = neighborhood_(root, pt, rad, level);
    indexArr nbhi;
    nbhi.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), nbhi.begin(),[](pointIndex x) { return x.second; });
    return nbhi;
}


KNN::KNN(const SiteVec &list) {
  pointVec points;
  for (auto &s : list) {
    point_t pt = {s.x, s.y};
    points.push_back(pt);
  }
  kd_tree_ = std::make_shared<KDTree>(points);
}

void KNN::NearestSite (Site &result, const Site &origin) const {
  point_t target = {origin.x, origin.y};
  auto res       = kd_tree_->nearest_point(target);
  result.x       = res[0];
  result.y       = res[1];
}

void KNN::NearestSite (SiteVec &result, const Site &origin, const double radius) const {
  result.clear();
  point_t target = {origin.x, origin.y};
  auto pt_arr = kd_tree_->neighborhood_points(target, radius);
  for (point_t &p : pt_arr) {
    Site ele(p[0], p[1]);
    result.push_back(ele);
  }
}

void KNN::InsertSite (const Site &to_insert) {
  point_t pt = {to_insert.x, to_insert.y};
  kd_tree_->insert_point(kd_tree_->root, pt);
}

} // namespace kdtree
} // namespace math
} // namespace toolbox
