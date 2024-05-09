#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;
namespace fast_planner {
PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh) {
  node = nh;

  // pubs_[0]
  traj_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 100);
  pubs_.push_back(traj_pub_);

  // pubs_[1]
  topo_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/topo_path", 100);
  pubs_.push_back(topo_pub_);

  // pubs_[2]
  predict_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/prediction", 100);
  pubs_.push_back(predict_pub_);

  // pubs_[3]
  visib_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/visib_constraint", 100);
  pubs_.push_back(visib_pub_);

  // pubs_[4]
  frontier_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/frontier", 1000);
  pubs_.push_back(frontier_pub_);

  // pubs_[5]
  yaw_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/yaw", 100);
  pubs_.push_back(yaw_pub_);

  // pubs_[6]
  yaw_array_pub_ = node.advertise<visualization_msgs::MarkerArray>("/planning_vis/yaw_array", 100);
  pubs_.push_back(yaw_array_pub_);

  // pubs_[7]
  viewpoint_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/viewpoints", 1000);
  pubs_.push_back(viewpoint_pub_);

  // pubs_[8]
  hgrid_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/hgrid", 1000);
  pubs_.push_back(hgrid_pub_);

  // pubs_[9]
  debug_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 1000);
  pubs_.push_back(debug_pub_);

  // pubs_[10]
  traj_bmk_pub_ =
      node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory_benchmark", 1000);
  pubs_.push_back(traj_bmk_pub_);

  // pubs_[11]
  yaw_array_bmk_pub_ =
      node.advertise<visualization_msgs::MarkerArray>("/planning_vis/yaw_array_benchmark", 100);
  pubs_.push_back(yaw_array_bmk_pub_);

  frontier_pcl_pub_ = node.advertise<sensor_msgs::PointCloud2>("/planning_vis/frontier_pcl", 1000);

  node.param("planning_vis/world_frame", world_frame_, string("world"));

  last_topo_path1_num_ = 0;
  last_topo_path2_num_ = 0;
  last_bspline_phase1_num_ = 0;
  last_bspline_phase2_num_ = 0;
  last_frontier_num_ = 0;
}

void PlanningVisualization::fillBasicInfo(visualization_msgs::Marker &mk,
                                          const Eigen::Vector3d &scale,
                                          const Eigen::Vector4d &color, const string &ns,
                                          const int &id, const int &shape) {
  mk.header.frame_id = world_frame_;
  mk.header.stamp = ros::Time::now();
  mk.id = id;
  mk.ns = ns;
  mk.type = shape;

  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = scale[0];
  mk.scale.y = scale[1];
  mk.scale.z = scale[2];
}

void PlanningVisualization::fillGeometryInfo(visualization_msgs::Marker &mk,
                                             const vector<Eigen::Vector3d> &list) {
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
}

void PlanningVisualization::fillGeometryInfo(visualization_msgs::Marker &mk,
                                             const vector<Eigen::Vector3d> &list1,
                                             const vector<Eigen::Vector3d> &list2) {
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
}

void PlanningVisualization::drawBox(const Eigen::Vector3d &center, const Eigen::Vector3d &scale,
                                    const Eigen::Vector4d &color, const string &ns, const int &id,
                                    const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  fillBasicInfo(mk, scale, color, ns, id, visualization_msgs::Marker::CUBE);
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  mk.pose.position.x = center[0];
  mk.pose.position.y = center[1];
  mk.pose.position.z = center[2];
  mk.action = visualization_msgs::Marker::ADD;

  pubs_[pub_id].publish(mk);
  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawText(const Eigen::Vector3d &pos, const string &text,
                                     const double &scale, const Eigen::Vector4d &color,
                                     const string &ns, const int &id, const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::TEXT_VIEW_FACING);

  // // clean old marker
  // mk.action = visualization_msgs::Marker::DELETE;
  // pubs_[pub_id].publish(mk);

  // pub new marker
  mk.text = text;
  mk.pose.position.x = pos[0];
  mk.pose.position.y = pos[1];
  mk.pose.position.z = pos[2];
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  ros::Duration(0.0005).sleep();
}

void PlanningVisualization::removeText(const string &ns, const int &id, const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d::Zero(), Color::Black(), ns, id,
                visualization_msgs::Marker::TEXT_VIEW_FACING);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);
}

void PlanningVisualization::drawSpheres(const vector<Eigen::Vector3d> &list, const double &scale,
                                        const Eigen::Vector4d &color, const string &ns,
                                        const int &id, const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::SPHERE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  // pub new marker
  fillGeometryInfo(mk, list);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawCubes(const vector<Eigen::Vector3d> &list, const double &scale,
                                      const Eigen::Vector4d &color, const string &ns, const int &id,
                                      const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::CUBE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  // pub new marker
  fillGeometryInfo(mk, list);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawLines(const vector<Eigen::Vector3d> &list1,
                                      const vector<Eigen::Vector3d> &list2, const double &scale,
                                      const Eigen::Vector4d &color, const string &ns, const int &id,
                                      const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  if (list1.empty() || list2.empty() || list1.size() != list2.size()) {
    // // ROS_WARN error details
    // ROS_WARN("drawLines error @ %s", ns.c_str());
    // if (list1.empty())
    //   ROS_WARN("list1 is empty");
    // if (list2.empty())
    //   ROS_WARN("list2 is empty");
    // if (list1.size() != list2.size())
    //   ROS_WARN("list1 and list2 are not the same size. list1: %d, list2: %d", int(list1.size()),
    //            int(list2.size()));
    return;
  }

  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::LINE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  if (list1.size() == 0)
    return;

  // pub new marker
  fillGeometryInfo(mk, list1, list2);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawLines(const vector<Eigen::Vector3d> &list, const double &scale,
                                      const Eigen::Vector4d &color, const string &ns, const int &id,
                                      const int &pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                visualization_msgs::Marker::LINE_LIST);

  // clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  pubs_[pub_id].publish(mk);

  if (list.size() == 0)
    return;

  // split the single list into two
  vector<Eigen::Vector3d> list1, list2;
  for (int i = 0; i < list.size() - 1; ++i) {
    list1.push_back(list[i]);
    list2.push_back(list[i + 1]);
  }

  // pub new marker
  fillGeometryInfo(mk, list1, list2);
  mk.action = visualization_msgs::Marker::ADD;
  pubs_[pub_id].publish(mk);
  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::displaySphereList(const vector<Eigen::Vector3d> &list,
                                              double resolution, const Eigen::Vector4d &color,
                                              int id, int pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  mk.header.frame_id = world_frame_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pubs_[pub_id].publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);
  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::displayCubeList(const vector<Eigen::Vector3d> &list, double resolution,
                                            const Eigen::Vector4d &color, int id, int pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  mk.header.frame_id = world_frame_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pubs_[pub_id].publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::displayLineList(const vector<Eigen::Vector3d> &list1,
                                            const vector<Eigen::Vector3d> &list2, double line_width,
                                            const Eigen::Vector4d &color, int id, int pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker mk;
  mk.header.frame_id = world_frame_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;
  pubs_[pub_id].publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = line_width;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::displayArrowList(const vector<Eigen::Vector3d> &list1,
                                             const vector<Eigen::Vector3d> &list2,
                                             double line_width, const Eigen::Vector4d &color,
                                             int id, int pub_id) {
  if (pubs_[pub_id].getNumSubscribers() == 0)
    return;

  visualization_msgs::MarkerArray markerArray;

  for (size_t i = 0; i < list1.size(); ++i) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = world_frame_;
    mk.header.stamp = ros::Time::now();
    mk.id = i;
    mk.type = visualization_msgs::Marker::ARROW;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = line_width;     // Arrow shaft diameter
    mk.scale.y = line_width * 3; // Arrow head diameter
    mk.scale.z = 0.0;            // Arrow head length (0.0 for auto-compute)
    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    geometry_msgs::Point pt;
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);
    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);

    markerArray.markers.push_back(mk);
  }

  pubs_[pub_id].publish(markerArray);

  // ros::Duration(0.0005).sleep();
}

void PlanningVisualization::drawBsplinesPhase1(vector<NonUniformBspline> &bsplines, double size) {
  vector<Eigen::Vector3d> empty;

  for (int i = 0; i < last_bspline_phase1_num_; ++i) {
    displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE + i % 100);
    displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE_CTRL_PT + i % 100);
  }
  last_bspline_phase1_num_ = bsplines.size();

  for (int i = 0; i < bsplines.size(); ++i) {
    drawBspline(bsplines[i], size, getColor(double(i) / bsplines.size(), 0.2), false, 2 * size,
                getColor(double(i) / bsplines.size()), false, 0.0, Eigen::Vector4d::Zero(), i);
  }
}

void PlanningVisualization::drawBsplinesPhase2(vector<NonUniformBspline> &bsplines, double size) {
  vector<Eigen::Vector3d> empty;

  for (int i = 0; i < last_bspline_phase2_num_; ++i) {
    drawSpheres(empty, size, Eigen::Vector4d(1, 0, 0, 1), "B-Spline", i, 0);
    drawSpheres(empty, size, Eigen::Vector4d(1, 0, 0, 1), "B-Spline", i + 50, 0);
    // displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE + (50
    // + i) % 100); displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1),
    // BSPLINE_CTRL_PT + (50 + i) % 100);
  }
  last_bspline_phase2_num_ = bsplines.size();

  for (int i = 0; i < bsplines.size(); ++i) {
    drawBspline(bsplines[i], size, getColor(double(i) / bsplines.size(), 0.6), false, 1.5 * size,
                getColor(double(i) / bsplines.size()), false, 0.0, Eigen::Vector4d::Zero(), i);
  }
}

void PlanningVisualization::drawBspline(NonUniformBspline &bspline, double size,
                                        const Eigen::Vector4d &color, bool show_ctrl_pts,
                                        double size2, const Eigen::Vector4d &color2,
                                        bool show_knot_pts, double size3,
                                        const Eigen::Vector4d &color3, int id1) {
  if (bspline.getControlPoint().size() == 0)
    return;

  // draw the b-spline trajectory
  vector<Eigen::Vector3d> traj_pts;
  double tm, tmp;
  bspline.getTimeSpan(tm, tmp);

  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
    traj_pts.push_back(pt);
  }
  // displaySphereList(traj_pts, size, color, BSPLINE + id1 % 100);
  drawSpheres(traj_pts, size, color, "B-Spline", id1, 0);

  // draw the control point
  if (show_ctrl_pts) {
    Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();
    vector<Eigen::Vector3d> ctp;
    for (int i = 0; i < int(ctrl_pts.rows()); ++i) {
      Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
      ctp.push_back(pt);
    }
    // displaySphereList(ctp, size2, color2, BSPLINE_CTRL_PT + id2 % 100);
    drawSpheres(ctp, size2, Color::Magenta(), "B-Spline", id1 + 50, 0);
  }

  // draw the knot point
  if (show_knot_pts) {
    vector<Eigen::Vector3d> knot_pts;
    bspline.getKnotPoint(knot_pts);
    // Write all knots into a txt file for benchmark
    // ofstream knot_file;
    // knot_file.open("/home/cindy/Downloads/icra2024/knots2.txt", ios::app);
    // for (int i = 0; i < int(knot_pts.size()); ++i) {
    //   knot_file << knot_pts[i](0) << " " << knot_pts[i](1) << " " << knot_pts[i](2) << endl;
    // }
    // knot_file.close();
    // ROS_WARN("Finish writing knots");
    drawSpheres(knot_pts, size3, color3, "B-Spline", id1 + 100, 0);
  }
}

void PlanningVisualization::drawBenchemarkPosTraj(const vector<Eigen::Vector3d> &pos, double size,
                                                  const Eigen::Vector4d &color) {
  if (pos.size() == 0)
    return;
  vector<Eigen::Vector3d> pos_draw;
  pos_draw.push_back(pos.front());
  for (int i = 0; i < pos.size() - 1; i++) {
    Eigen::Vector3d interpolated = (pos[i] + pos[i + 1]) / 2.0;
    pos_draw.push_back(interpolated);
    pos_draw.push_back(pos[i + 1]);
  }
  // displaySphereList(traj_pts, size, color, BSPLINE + id1 % 100);
  drawSpheres(pos_draw, size, Color::DeepGreen(), "Benchmark", 0, PUBLISHER::TRAJECTORY_BENCHMARK);
}

void PlanningVisualization::drawBenchemarkYawTraj(const vector<Eigen::Vector3d> &pos,
                                                  const vector<Eigen::Vector3d> &acc,
                                                  const vector<double> &yaw, double &knot_span) {
  vector<Eigen::Vector3d> pos_draw, acc_draw;
  vector<double> yaw_draw;

  double duration = 0.02 / 0.92 * (pos.size() - 1);
  int num_knots = floor(duration / knot_span) + 1;
  vector<double> knot_vector;
  for (int i = 0; i < num_knots; i++) {
    knot_vector.push_back(i * knot_span);
  }
  vector<int> index_set;
  for (double knot_time : knot_vector) {
    knot_time += 1e-4;
    int i = 0;
    while (i * 0.02 / 0.92 < knot_time) {
      i++;
    }
    index_set.push_back(i - 1);
  }
  for (int i : index_set) {
    pos_draw.push_back(pos[i]);
    acc_draw.push_back(acc[i]);
    yaw_draw.push_back(yaw[i]);
  }

  vector<Eigen::Vector3d> arrow_end_pts;
  for (int i = 0; i < yaw_draw.size(); ++i) {
    Eigen::Vector3d plane_yaw_dir(cos(yaw_draw[i]), sin(yaw_draw[i]), 0);
    Eigen::Vector3d g(0, 0, -9.8);

    Eigen::Vector3d thrust, temp, yaw_dir, end_pt;
    thrust = acc_draw[i] - g;
    temp = thrust.cross(plane_yaw_dir);
    yaw_dir = temp.cross(thrust).normalized();
    end_pt = pos_draw[i] + 1.0 * yaw_dir;
    arrow_end_pts.push_back(end_pt);
  }

  displayArrowList(pos_draw, arrow_end_pts, 0.05, Color::DeepGreen(), 1, PUBLISHER::YAW_TRAJ_ARRAY_BENCHMARK);
}

void PlanningVisualization::drawTopoGraph(list<GraphNode::Ptr> &graph, double point_size,
                                          double line_width, const Eigen::Vector4d &color1,
                                          const Eigen::Vector4d &color2,
                                          const Eigen::Vector4d &color3, int id) {
  // clear exsiting node and edge (drawn last time)
  vector<Eigen::Vector3d> empty;
  displaySphereList(empty, point_size, color1, GRAPH_NODE, 1);
  displaySphereList(empty, point_size, color1, GRAPH_NODE + 50, 1);
  displayLineList(empty, empty, line_width, color3, GRAPH_EDGE, 1);

  /* draw graph node */
  vector<Eigen::Vector3d> guards, connectors;
  for (list<GraphNode::Ptr>::iterator iter = graph.begin(); iter != graph.end(); ++iter) {
    if ((*iter)->type_ == GraphNode::Guard) {
      guards.push_back((*iter)->pos_);
    } else if ((*iter)->type_ == GraphNode::Connector) {
      connectors.push_back((*iter)->pos_);
    }
  }
  displaySphereList(guards, point_size, color1, GRAPH_NODE, 1);
  displaySphereList(connectors, point_size, color2, GRAPH_NODE + 50, 1);

  /* draw graph edge */
  vector<Eigen::Vector3d> edge_pt1, edge_pt2;
  for (list<GraphNode::Ptr>::iterator iter = graph.begin(); iter != graph.end(); ++iter) {
    for (int k = 0; k < (*iter)->neighbors_.size(); ++k) {
      edge_pt1.push_back((*iter)->pos_);
      edge_pt2.push_back((*iter)->neighbors_[k]->pos_);
    }
  }
  displayLineList(edge_pt1, edge_pt2, line_width, color3, GRAPH_EDGE, 1);
}

void PlanningVisualization::drawTopoPathsPhase2(vector<vector<Eigen::Vector3d>> &paths,
                                                double line_width) {
  // clear drawn paths
  Eigen::Vector4d color1(1, 1, 1, 1);
  for (int i = 0; i < last_topo_path1_num_; ++i) {
    vector<Eigen::Vector3d> empty;
    displayLineList(empty, empty, line_width, color1, SELECT_PATH + i % 100, 1);
    displaySphereList(empty, line_width, color1, PATH + i % 100, 1);
  }

  last_topo_path1_num_ = paths.size();

  // draw new paths
  for (int i = 0; i < paths.size(); ++i) {
    vector<Eigen::Vector3d> edge_pt1, edge_pt2;

    for (int j = 0; j < paths[i].size() - 1; ++j) {
      edge_pt1.push_back(paths[i][j]);
      edge_pt2.push_back(paths[i][j + 1]);
    }

    displayLineList(edge_pt1, edge_pt2, line_width, getColor(double(i) / (last_topo_path1_num_)),
                    SELECT_PATH + i % 100, 1);
  }
}

void PlanningVisualization::drawTopoPathsPhase1(vector<vector<Eigen::Vector3d>> &paths,
                                                double size) {
  // clear drawn paths
  Eigen::Vector4d color1(1, 1, 1, 1);
  for (int i = 0; i < last_topo_path2_num_; ++i) {
    vector<Eigen::Vector3d> empty;
    displayLineList(empty, empty, size, color1, FILTERED_PATH + i % 100, 1);
  }

  last_topo_path2_num_ = paths.size();

  // draw new paths
  for (int i = 0; i < paths.size(); ++i) {
    vector<Eigen::Vector3d> edge_pt1, edge_pt2;

    for (int j = 0; j < paths[i].size() - 1; ++j) {
      edge_pt1.push_back(paths[i][j]);
      edge_pt2.push_back(paths[i][j + 1]);
    }

    displayLineList(edge_pt1, edge_pt2, size, getColor(double(i) / (last_topo_path2_num_), 0.2),
                    FILTERED_PATH + i % 100, 1);
  }
}

void PlanningVisualization::drawGoal(Eigen::Vector3d goal, double resolution,
                                     const Eigen::Vector4d &color, int id) {
  vector<Eigen::Vector3d> goal_vec = {goal};
  displaySphereList(goal_vec, resolution, color, GOAL + id % 100);
}

void PlanningVisualization::drawGeometricPath(const vector<Eigen::Vector3d> &path,
                                              double resolution, const Eigen::Vector4d &color,
                                              int id) {
  displaySphereList(path, resolution, color, PATH + id % 100);
}

void PlanningVisualization::drawPolynomialTraj(PolynomialTraj poly_traj, double resolution,
                                               const Eigen::Vector4d &color, int id) {
  vector<Eigen::Vector3d> poly_pts;
  poly_traj.getSamplePoints(poly_pts);
  displaySphereList(poly_pts, resolution, color, POLY_TRAJ + id % 100);
}

void PlanningVisualization::drawPrediction(ObjPrediction pred, double resolution,
                                           const Eigen::Vector4d &color, int id) {
  ros::Time time_now = ros::Time::now();
  double start_time = (time_now - ObjHistory::global_start_time_).toSec();
  const double range = 5.6;

  vector<Eigen::Vector3d> traj;
  for (int i = 0; i < pred->size(); i++) {
    PolynomialPrediction poly = pred->at(i);
    if (!poly.valid())
      continue;

    for (double t = start_time; t <= start_time + range; t += 0.8) {
      Eigen::Vector3d pt = poly.evaluateConstVel(t);
      traj.push_back(pt);
    }
  }
  displaySphereList(traj, resolution, color, id % 100, 2);
}

void PlanningVisualization::drawVisibConstraint(const Eigen::MatrixXd &ctrl_pts,
                                                const vector<Eigen::Vector3d> &block_pts) {
  int visible_num = ctrl_pts.rows() - block_pts.size();

  /* draw block points, their projection rays and visible pairs */
  vector<Eigen::Vector3d> pts1, pts2, pts3, pts4;
  int n = ctrl_pts.rows() - visible_num;

  for (int i = 0; i < n; ++i) {
    Eigen::Vector3d qb = block_pts[i];

    if (fabs(qb[2] + 10086) > 1e-3) {
      // compute the projection
      Eigen::Vector3d qi = ctrl_pts.row(i);
      Eigen::Vector3d qj = ctrl_pts.row(i + visible_num);
      Eigen::Vector3d dir = (qj - qi).normalized();
      Eigen::Vector3d qp = qi + dir * ((qb - qi).dot(dir));

      pts1.push_back(qb);
      pts2.push_back(qp);
      pts3.push_back(qi);
      pts4.push_back(qj);
    }
  }

  displayCubeList(pts1, 0.1, Eigen::Vector4d(1, 1, 0, 1), 0, 3);
  displaySphereList(pts4, 0.2, Eigen::Vector4d(0, 1, 0, 1), 1, 3);
  displayLineList(pts1, pts2, 0.015, Eigen::Vector4d(0, 1, 1, 1), 2, 3);
  displayLineList(pts3, pts4, 0.015, Eigen::Vector4d(0, 1, 0, 1), 3, 3);
}

void PlanningVisualization::drawVisibConstraint(const Eigen::MatrixXd &pts,
                                                const vector<VisiblePair> &pairs) {
  vector<Eigen::Vector3d> pts1, pts2, pts3, pts4;
  for (auto pr : pairs) {
    Eigen::Vector3d qb = pr.qb_;
    Eigen::Vector3d qi = pts.row(pr.from_);
    Eigen::Vector3d qj = pts.row(pr.to_);
    Eigen::Vector3d dir = (qj - qi).normalized();
    Eigen::Vector3d qp = qi + dir * ((qb - qi).dot(dir));
    pts1.push_back(qb);
    pts2.push_back(qp);
    pts3.push_back(qi);
    pts4.push_back(qj);
  }
  displayCubeList(pts1, 0.1, Eigen::Vector4d(1, 1, 0, 1), 0, 3);
  displaySphereList(pts4, 0.2, Eigen::Vector4d(0, 1, 0, 1), 1, 3);
  displayLineList(pts1, pts2, 0.015, Eigen::Vector4d(0, 1, 1, 1), 2, 3);
  displayLineList(pts3, pts4, 0.015, Eigen::Vector4d(0, 1, 0, 1), 3, 3);
}

void PlanningVisualization::drawViewConstraint(const ViewConstraint &vc) {
  if (vc.idx_ < 0)
    return;
  visualization_msgs::Marker mk;
  mk.header.frame_id = world_frame_;
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.type = visualization_msgs::Marker::ARROW;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.scale.x = 0.1;
  mk.scale.y = 0.2;
  mk.scale.z = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.5;
  mk.color.b = 0.0;
  mk.color.a = 1.0;

  geometry_msgs::Point pt;
  pt.x = vc.pt_[0];
  pt.y = vc.pt_[1];
  pt.z = vc.pt_[2];
  mk.points.push_back(pt);
  pt.x = vc.pt_[0] + vc.dir_[0];
  pt.y = vc.pt_[1] + vc.dir_[1];
  pt.z = vc.pt_[2] + vc.dir_[2];
  mk.points.push_back(pt);
  pubs_[3].publish(mk);

  vector<Eigen::Vector3d> pts = {vc.pcons_};
  displaySphereList(pts, 0.2, Eigen::Vector4d(0, 1, 0, 1), 1, 3);
}

void PlanningVisualization::drawFrontier(const vector<vector<Eigen::Vector3d>> &frontiers) {
  for (int i = 0; i < frontiers.size(); ++i) {
    // displayCubeList(frontiers[i], 0.1, getColor(double(i) / frontiers.size(),
    // 0.4), i, 4);
    drawCubes(frontiers[i], 1.0, getColor(double(i) / frontiers.size(), 0.8), "frontier", i,
              PUBLISHER::FRONTIER);
  }

  vector<Eigen::Vector3d> frontier;
  for (int i = frontiers.size(); i < last_frontier_num_; ++i) {
    // displayCubeList(frontier, 0.1, getColor(1), i, 4);
    drawCubes(frontier, 1.0, getColor(1), "frontier", i, PUBLISHER::FRONTIER);
  }
  last_frontier_num_ = frontiers.size();
}

void PlanningVisualization::drawYawTraj(NonUniformBspline &pos, NonUniformBspline &yaw,
                                        const double &dt) {
  double duration = pos.getTimeSum();
  vector<Eigen::Vector3d> pts1, pts2;

  for (double tc = 0.0; tc <= duration + 1e-3; tc += dt) {
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    pc[2] += 0.15;
    double yc = yaw.evaluateDeBoorT(tc)[0];
    Eigen::Vector3d dir(cos(yc), sin(yc), 0);
    Eigen::Vector3d pdir = pc + 1.0 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  displayLineList(pts1, pts2, 0.02, Color::Magenta(), 0, PUBLISHER::YAW_TRAJ);
}

void PlanningVisualization::drawYawPath(NonUniformBspline &pos, const vector<double> &yaw,
                                        const double &dt) {
  vector<Eigen::Vector3d> pts1, pts2;

  for (int i = 0; i < yaw.size(); ++i) {
    Eigen::Vector3d pc = pos.evaluateDeBoorT(i * dt);
    pc[2] += 0.3;
    Eigen::Vector3d dir(cos(yaw[i]), sin(yaw[i]), 0);
    Eigen::Vector3d pdir = pc + 0.5 * dir;
    pts1.push_back(pc);
    pts2.push_back(pdir);
  }
  displayLineList(pts1, pts2, 0.02, Color::Magenta(), 1, PUBLISHER::YAW_TRAJ);
}

void PlanningVisualization::drawYawPath(const vector<Eigen::Vector3d> &pos,
                                        const vector<double> &yaw) {
  vector<Eigen::Vector3d> pts2;

  for (int i = 0; i < yaw.size(); ++i) {
    Eigen::Vector3d dir(cos(yaw[i]), sin(yaw[i]), 0);
    Eigen::Vector3d pdir = pos[i] + 1.0 * dir;
    pts2.push_back(pdir);
  }
  displayLineList(pos, pts2, 0.02, Color::Magenta(), 1, PUBLISHER::YAW_TRAJ);
}

void PlanningVisualization::drawYawOnKnots(NonUniformBspline &pos, NonUniformBspline &acc,
                                           NonUniformBspline &yaw) {
  if (pos.getControlPoint().size() == 0 || yaw.getControlPoint().size() == 0)
    return;

  vector<Eigen::Vector3d> pos_knot_pts, acc_knot_pts, yaw_knot_pts;
  pos.getKnotPoint(pos_knot_pts);
  acc.getKnotPoint(acc_knot_pts);
  yaw.getKnotPoint(yaw_knot_pts);

  vector<Eigen::Vector3d> arrow_end_pts;
  for (int i = 0; i < yaw_knot_pts.size(); ++i) {
    Eigen::Vector3d plane_yaw_dir(cos(yaw_knot_pts[i](0)), sin(yaw_knot_pts[i](0)), 0);
    Eigen::Vector3d g(0, 0, -9.8);

    Eigen::Vector3d thrust, temp, yaw_dir, end_pt;
    thrust = acc_knot_pts[i] - g;
    temp = thrust.cross(plane_yaw_dir);
    yaw_dir = temp.cross(thrust).normalized();
    end_pt = pos_knot_pts[i] + 0.75 * yaw_dir;
    arrow_end_pts.push_back(end_pt);
  }

  // displayLineList(pos_knot_pts, yaw_dir, 0.02, Color::Magenta(), 1, PUBLISHER::YAW_TRAJ);
  displayArrowList(pos_knot_pts, arrow_end_pts, 0.05, Color::Pink(), 1,
                   PUBLISHER::YAW_TRAJ_ARRAY);
}

Eigen::Vector4d PlanningVisualization::getColor(const double &h, double alpha) {
  double h1 = h;
  if (h1 < 0.0) {
    h1 = 0.0;
  }

  double lambda;
  Eigen::Vector4d color1, color2;
  if (h1 >= -1e-4 && h1 < 1.0 / 6) {
    lambda = (h1 - 0.0) * 6;
    color1 = Eigen::Vector4d(1, 0, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 1, 1);
  } else if (h1 >= 1.0 / 6 && h1 < 2.0 / 6) {
    lambda = (h1 - 1.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 0, 1, 1);
  } else if (h1 >= 2.0 / 6 && h1 < 3.0 / 6) {
    lambda = (h1 - 2.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 1, 1);
  } else if (h1 >= 3.0 / 6 && h1 < 4.0 / 6) {
    lambda = (h1 - 3.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 0, 1);
  } else if (h1 >= 4.0 / 6 && h1 < 5.0 / 6) {
    lambda = (h1 - 4.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);
  } else if (h1 >= 5.0 / 6 && h1 <= 1.0 + 1e-4) {
    lambda = (h1 - 5.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3) = alpha;

  return fcolor;
}

void PlanningVisualization::drawFrontierPointcloud(
    const vector<vector<Eigen::Vector3d>> &frontiers) {
  if (frontier_pcl_pub_.getNumSubscribers() == 0)
    return;

  // TicToc t_pcl;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

  for (int i = 0; i < frontiers.size(); ++i) {
    auto &frontier = frontiers[i];
    for (auto pt : frontier) {
      pcl::PointXYZRGBA p;
      p.x = pt[0];
      p.y = pt[1];
      p.z = pt[2];

      Eigen::Vector4d color = getColor(double(i) / frontiers.size(), 0.4);
      p.r = color[0] * 255;
      p.g = color[1] * 255;
      p.b = color[2] * 255;
      p.a = color[3] * 255;
      pointcloud->points.push_back(p);
    }
  }

  // t_pcl.toc("draw frontier pointcloud");

  pointcloud->header.frame_id = world_frame_;
  pointcloud->width = pointcloud->points.size();
  pointcloud->height = 1;
  pointcloud->is_dense = true; // True if no invalid points

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*pointcloud, pointcloud_msg);
  frontier_pcl_pub_.publish(pointcloud_msg);
}

} // namespace fast_planner