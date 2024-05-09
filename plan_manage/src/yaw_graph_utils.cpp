// #include <active_perception/yaw_graph_utils.h>
#include <plan_manage/yaw_graph_utils.h>

namespace fast_planner {

/* ----------------------------- class Graph ---------------------------- */

void Graph::print() {
  for (auto v : vertice_) {
    v->print();
    v->printNeighbors();
  }
}

void Graph::addVertex(const YawVertex::Ptr &vertex) { vertice_.push_back(vertex); }

void Graph::addEdge(const int &from, const int &to, const double &gain) {
  shared_ptr<YawEdge> edge(new YawEdge(gain, vertice_[to]));
  vertice_[from]->edges_.push_back(edge);
  // vertice_[from]->neighbors_.push_back(vertice_[to]);
}

void Graph::setParams(const double &w, const double &my, const double &dt) {
  w_ = w;
  max_yaw_rate_ = my;
  dt_ = dt;
}

double Graph::penal(const double &diff) {
  double yr = diff / dt_;
  if (yr <= max_yaw_rate_) {
    return 0.0;
  } else {
    // double penal = pow(yr - max_yaw_rate_, 2);
    // ROS_INFO("yaw diff: %f, yaw rate: %f, max yaw rate: %f, penal: %f", diff, yr, max_yaw_rate_,
    // penal);
    return pow(yr - max_yaw_rate_, 2);
  }
}

void Graph::dijkstraSearch(const int &start, const int &goal, vector<YawVertex::Ptr> &path) {
  YawVertex::Ptr start_v = vertice_[start];
  YawVertex::Ptr end_v = vertice_[goal];
  start_v->g_value_ = 0.0;

  queue<YawVertex::Ptr> open_set;
  unordered_map<int, int> open_set_map;
  unordered_map<int, int> close_set;
  open_set.push(start_v);
  open_set_map[start_v->id_] = 1;

  while (!open_set.empty()) {
    auto vc = open_set.front();
    open_set.pop();
    open_set_map.erase(vc->id_);
    close_set[vc->id_] = 1;

    // reach target
    if (vc == end_v) {
      YawVertex::Ptr vit = vc;
      while (vit != nullptr) {
        if (!vit->virtual_)
          path.push_back(vit);
        vit = vit->parent_;
      }
      reverse(path.begin(), path.end());
      // ROS_WARN("Dijkstra reach target!");
      // for (int i = 0; i < path.size(); i++) {
      //   ROS_INFO("Path[%d]: virtual: %d, pos: (%f, %f, %f), yaw: %f", i, path[i]->virtual_,
      //            path[i]->pos_(0), path[i]->pos_(1), path[i]->pos_(2), path[i]->yaw_);
      // }
      // for (int i = 0; i < path.size() - 1; i++) {
      //   ROS_INFO("--- Layer %d -> Layer %d ---", i, i + 1);
      //   int gid = path[i + 1]->id_;
      //   for (YawEdge::Ptr e : path[i]->edges_) {
      //     if (e->next_vertex_->id_ == gid) {
      //       ROS_INFO("SELECTED GAIN: %f", e->gain_);
      //     } else
      //       ROS_INFO("gain: %f", e->gain_);
      //   }
      // }
      return;
    }
    // auto nbs = vc->neighbors_;
    for (YawEdge::Ptr e : vc->edges_) {
      // skip vertex in close set
      YawVertex::Ptr vb = e->next_vertex_;
      if (close_set.find(vb->id_) != close_set.end())
        continue;

      // update new or open vertex
      // double g_tmp = vc->g_value_ - vc->gain(vb) + w_ * penal(vc->dist(vb));
      double g_tmp = vc->g_value_ - e->gain_ + w_ * penal(vc->dist(vb));
      if (open_set_map.find(vb->id_) == open_set_map.end()) {
        open_set_map[vb->id_] = 1;
        open_set.push(vb);
      } else if (g_tmp > vb->g_value_) {
        continue;
      }
      vb->parent_ = vc;
      vb->g_value_ = g_tmp;
    }
  }

  ROS_ERROR("Dijkstra can't find path!");
  ROS_ASSERT(false);
}
} // namespace fast_planner