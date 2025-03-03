// Conventional expansion methods may lead to frequent or unnecessary turns;
// First, calculate the shortest polyline path from the start to the goal as a reference path;
// Equidistantly select multiple sub-goal poses on the reference path, where the orientation of each sub-goal pose is the direction of the reference line at that point;
// Use hybrid search; additionally, during the search process, it will prioritize attempting to hit sub-goal points. If a sub-goal is hit, it continues to hit subsequent sub-goals. If it cannot hit a sub-goal, it uses the traditional hybrid expansion method until it successfully reaches the goal pose;

// In the above operations, hitting sub-goal points can effectively reduce the number of node expansions in open areas or long straight segments of the reference line, speeding up the search to the goal pose;

// Add a bubble zone during the search to limit the search space
// Draw the area on the path [local corridor/bubble zone], with a footprint of 5 car widths and 40 pixels, only considering the path footprint range, to optimize Voronoi generation time
        // TODO: Change this to dynamic allocation to prevent the image from being too large
        {
            cv::Mat mask = cv::Mat::zeros(planMapCv_raw.rows, planMapCv_raw.cols, CV_8UC1);
            planMapCv    = planMapCv_raw.clone();
            planMapCv.setTo(0);
            // cv::line(mask, cv::Point(333,6666), cv::Point(200,200),cv::Scalar(255,255,255),40,8,0);
            for (unsigned int i = 0; i < astarPath.size() - 1; i++) {
                cv::line(mask, cv::Point(astarPath[i].X, astarPath[i].Y),
                         cv::Point(astarPath[i + 1].X, astarPath[i + 1].Y),
                         cv::Scalar(255, 255, 255), 20 * 4, 8, 0); // This draws rounded lines, equivalent to circles; astarPath points are sparse
            }
            // TODO: Note to draw black lines on the edges
            // cv::imwrite("mask.bmp", mask); // Reading and writing files is time-consuming (to delete)
            planMapCv_raw.copyTo(planMapCv, mask);
            // cv::imwrite("planMapCv.bmp", planMapCv); // Reading and writing files is time-consuming (to delete)
        }


/*[key] Generation of multiple sub-goals
Select long straight segments on the shortest polyline and sample at intervals;
Equidistantly select multiple sub-goal poses on the reference path, where the orientation of each sub-goal pose is the direction of the reference line at that point;*/

// Step: Calculate sub-goal points
        std::vector<Pose2D> sub_goals;
        do {
            // Sample long straight segments in astarPath
            const double k_subGoal_dis_interval =  0.5f;
            // Sample a path point every 0.5m
            const double K_cut_segment_thresh = 1.f; 
            // Only sample segments with a length of at least 1m
            std::vector<Pose2D> pose_sparse_list; 
            // Original path points (poses), in physical coordinates
            for (auto item : astarPath) {
                pose_sparse_list.push_back(Pose2D(mapData.mapParam.idx2x(item.X), mapData.mapParam.idx2y(item.Y), 0));
            }

            for (size_t i = 0; i < pose_sparse_list.size() - 1; i++) {
                float dist = pose_sparse_list[i].GetDistance(pose_sparse_list[i+1]);
                if (dist < K_cut_segment_thresh || dist < k_subGoal_dis_interval) {
                    continue;
                }
                LineSegment2d segment({pose_sparse_list[i].X, pose_sparse_list[i].Y}, {pose_sparse_list[i + 1].X, pose_sparse_list[i + 1].Y});
                Pose2D segment_start; 
                // Starting pose on the segment
                segment_start.X = segment.start().x();
                segment_start.Y = segment.start().y();
                segment_start.SetPhi(segment.heading());
                int num_inserts =  std::ceil(dist / k_subGoal_dis_interval) - 1;
                for (int j = 1; j <= num_inserts; ++j) {
                    float ratio = static_cast<float>(j) / (num_inserts + 1);
                    // new_poses.push_back(interpolate(path.poses[i], path.poses[i + 1], ratio));
                    sub_goals.push_back(segment_start + Pose2D(ratio * segment.length(), 0, 0)); 
                    // Calculate sub-goal points on the segment
                }
            }
            // poseEnd
            sub_goals.push_back(poseEnd);
        } while (0);


// Code implementation for hitting a single sub-goal in a single attempt

bool hybrid_a_star::DubinsShotSubGoal(Node *current_node, Pose2D subGoal, float real_dubins_radius, Node **newNode)
{
    Pose2D start = GetNodeSegEnd(current_node);
    Pose2D endPose = subGoal;
    // start
    double q0[] = {start.X, start.Y, start.Phi()};
    if (q0[2] < 0) {
        q0[2] = WrapTo2Pi(q0[2]);
    }
    // goal
    double q1[] = {endPose.X, endPose.Y, endPose.Phi()};
    if (q1[2] < 0) {
        q1[2] = WrapTo2Pi(q1[2]);
    }
    // initialize the path
    DubinsSimple::DubinsPath path;
    // calculate the path
    int res = DubinsSimple::dubins_init(q0, q1, real_dubins_radius, &path);
    if (res != 0) {
        return false;
    }
    std::vector<Pose2D> path_lists;
    float x = 0.f;
    // x += k_dubins_step_size;  
    // Prevent overlapping path points, but it doesn't matter if they overlap. The pose angle of each path point is calculated independently and is unrelated to the next path point.
    float length = dubins_path_length(&path);
    if (length < 0.01f) {
        return false;
    }
    while (x <= length) {
        double q[3];
        dubins_path_sample(&path, x, q); 
        // Sample points on the curve at intervals of length x
        Pose2D pose(q[0], q[1], WrapToPi(q[2]));
        // collision check
        if (CheckPoseCollisionSafe(pose) == false) {
            return false;
        }
        path_lists.push_back(pose);
        x += k_dubins_step_size;
    }
    {
        int index         = Pose2Index(path_lists.back());
        double added_cost = 0.0;
        added_cost += STEER_COST * fabs(0);                              
        // Steering angle, curvature cost. Increasing this term can reduce sharp curvatures. [When the steering angle aligns with the A* trend, reduce this term to follow the A* path's steering trend.]
        // Note: TODO: It's better to calculate the steering angle using real_dubins_radius.
        // added_cost += STEER_CHANGE_COST * fabs(WrapToPi(path_lists.back().Phi() - start.Phi())); 
        // Steering change cost. Increasing this term can reduce frequent turns. [This can be adjusted based on the A* path to avoid steering changes in long straight segments and allow changes during turns.]
        double cost        = current_node->cost ;
        // + added_cost ;//length * 10; 
        // Here, arc_len should preferably be multiplied by a coefficient. arc_len is used to optimize the path length.
        auto neighbor_node = new Node(index, start, length, 0, current_node->state_grid_index, cost, true);
        // Record the Dubins path when the sub-goal is hit.
        *newNode = neighbor_node;
        mSubGoalPaths[neighbor_node] = path_lists;
    }
    return true;
}

/*[Key] Hitting multiple sub-goals
Prioritize attempting to hit sub-goal points. If a sub-goal is hit, continue to hit subsequent sub-goals. If unable to hit a sub-goal, use the traditional hybrid expansion method until successfully reaching the goal pose.
Use multiple radii for hitting, prioritizing larger radii.
Note: Each attempt to hit a sub-goal involves collision detection, which can be time-consuming.
To optimize time, sub-goals are within a 10-meter range, and the farthest one is prioritized for hitting. */
// Hit sub-goals
        // TODO: Optimization: If the start and end poses are very close, only use the end pose as the sub-goal point, and do not attempt to hit other intermediate points.
        {
            valid_sub_goal = false;
            // Determine the range of sub-goal points;
            int index_fathest = mSubGoals.size() - 1; 
            // The index of the farthest sub-goal point currently being searched
            {
                // Sub-goal points are within a 10-meter range
                const float k_SubGoalDisInReach = 10.f; 
                int index = std::max(sub_goal_counter, 0);
                float dis_sum = 0;
                do {
                    if (index == mSubGoals.size() - 1) {
                        break;
                    }
                    index++;
                    if (index == mSubGoals.size() - 1) {
                        break;
                    }
                    dis_sum += mSubGoals[index].GetDistance(mSubGoals[index - 1]);
                    if (dis_sum > k_SubGoalDisInReach) {
                        break;
                    }
                } while (1);
                index_fathest = index;
            }
            // Sub-goal points are within a 10-meter range
            for (int i = index_fathest /*mSubGoals.size() - 1*/; i > sub_goal_counter; --i) {
                Pose2D sub_goal = mSubGoals[i];
                // Use Dubins to connect and check if it hits
                do {
                    // bool hybrid_a_star::DubinsShotSubGoal(Node *current_node, Pose2D subGoal, float real_dubins_radius, Node *newNode)
                    Node *newNode = nullptr;
                    // Use multiple radii for hitting, prioritizing larger radii
                    bool res = false;
                    for (float dubins_radius = k_dubins_radius_max; dubins_radius >= k_dubins_radius_min; dubins_radius -= k_dubins_radius_add_step) {
                        res = DubinsShotSubGoal(current_node_ptr, sub_goal, dubins_radius, &newNode); 
                        // k_dubins_radius_min
                        if (res == true) {
                            break; 
                            // Prioritize hitting and then exit
                        }
                    }
                    // If not hit, try another target point
                    if (res == false) {
                        break;
                    }
                    if (mCloseList.find(newNode->state_grid_index) != mCloseList.end()) {
                        // Already searched and expanded
                        delete newNode;
                        break;
                    }
                    if (mOpenList.find(newNode->state_grid_index) == mOpenList.end()) {
                        // Calculate G + H
                        // Directly hit the goal, end the search
                        if (i == mSubGoals.size() - 1) {
                            m_terminal_node_ptr_ = newNode;
                            return true; 
                            // Hit the goal
                        }
                        double f_cost = newNode->cost + ComputeH(newNode);
                        mOpenList.insert(std::make_pair(newNode->state_grid_index, newNode));
                        openset_.insert(std::make_pair((uint64_t)(f_cost * 1000), newNode->state_grid_index));
                    } else if (mOpenList.find(newNode->state_grid_index) != mOpenList.end()) {
                        delete newNode;
                        break;
                    }
                    sub_goal_counter = i;
                    valid_sub_goal   = true; 
                    // Can directly hit the Voronoi node using Dubins
                } while(0);
            }
            if (valid_sub_goal) {
                continue;
            }
        } // End of hitting sub-goals

        // Expand nodes
        GetNeighborNodes(current_node_ptr, neighbor_nodes);
        for (unsigned int i = 0; i < neighbor_nodes.size(); ++i) {
            neighbor_node_ptr = neighbor_nodes[i];
            int neighbor_node_index = neighbor_node_ptr->state_grid_index;
            // Pose2Index(GetNodeSegEnd(neighbor_node_ptr));
            if(mCloseList.find(neighbor_node_index) != mCloseList.end()) {
                // Already searched and expanded
                delete neighbor_node_ptr;
                continue;
            }
            if (mOpenList.find(neighbor_node_index) == mOpenList.end()  ) {
                // Calculate G + H
                double f_cost = neighbor_node_ptr->cost + ComputeH(neighbor_node_ptr);
                mOpenList.insert(std::make_pair(neighbor_node_ptr->state_grid_index, neighbor_node_ptr));
                openset_.insert(std::make_pair((uint64_t)(f_cost * 1000), neighbor_node_ptr->state_grid_index));
            } else if (mOpenList.find(neighbor_node_index) != mOpenList.end()) {
                // Compare G values
                // if (neighbor_node_ptr->cost < mOpenList[neighbor_node_index]->cost) {
                //     delete mOpenList[neighbor_node_index];
                //     double f_cost = neighbor_node_ptr->cost + ComputeH(neighbor_node_ptr);
                //     mOpenList.insert(std::make_pair(neighbor_node_ptr->state_grid_index, neighbor_node_ptr));
                //     mOpenList[neighbor_node_ptr->state_grid_index] = neighbor_node_ptr;
                //     openset_.insert(std::make_pair(f_cost*1000, neighbor_node_ptr->state_grid_index));
                // } else {
                    delete neighbor_node_ptr;
                // }
            }
        }



// Test code
// Select multiple random poses for testing

// Select multiple random poses
void testHybridMany()
{
    auto copyFileWithIndex = [](int i) -> void {
        std::string sourceFilePath = "/tmp/debug_record/hybrid_canvas.png"; 
        // hybrid_canvas.png
        std::string destinationFilePath;
        // Construct the destination file path
        std::stringstream ss;
        ss << sourceFilePath.substr(0, sourceFilePath.find_last_of(".")) << "_" << i << sourceFilePath.substr(sourceFilePath.find_last_of("."));
        ss >> destinationFilePath;

        // Open the source and destination files
        std::ifstream sourceFile(sourceFilePath, std::ios::binary);
        std::ofstream destinationFile(destinationFilePath, std::ios::binary);

        // Copy the file content
        destinationFile << sourceFile.rdbuf();

        // Close the files
        sourceFile.close();
        destinationFile.close();
    };
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-2, 2);
    std::uniform_real_distribution<double> distribution_angle(-3, 3);
    for (int i = 0; i < 100; i++) {
        TestHybrid test;
        test.SetVisulization(true);
        TMapData mapData({-10, -10, 50 * 20, 50 * 20}, 255);
        for (float x = 0; x < 35; x += 6) {
            for (float y = 0; y < 35; y += 6) {
                int xInt = mapData.x2idx(x);
                int yInt = mapData.y2idx(y);
                LOCALCV::CCommonAlg::circleMat(0, xInt + RandomInt(10, 20), yInt + RandomInt(10, 20), 10 + RandomInt(10, 40), &mapData.map[0], mapData.mapParam.width, mapData.mapParam.height,
                                               0, mapData.mapParam.width, 0, mapData.mapParam.height);
            }
        }
        Pose2D startPose(-5 + distribution(generator), 0 + distribution(generator), 1.7 + distribution_angle(generator));
        Pose2D endPose(35 + distribution(generator), 32 + distribution(generator), -1.7 + distribution_angle(generator));
        bool res = test.test(mapData, startPose, endPose);
        FLOGD("test hybrid result is %d", res);
        if (res == false) {
            continue;
        }
        copyFileWithIndex(i); 
        // Save the result with a new name
    }
}


// Improvements
// Additionally, reverse search can solve the problem of difficulty in hitting the goal when it is near a wall or corner, improving efficiency and success rate.
// Additionally, if expanding from the start pose, the close set is considered trapped if the number of nodes within a certain radius reaches a specified limit and cannot expand further. Similarly, if expanding from the goal pose, the close set is considered trapped if the number of nodes within a certain radius reaches a specified limit and cannot expand further.

// Input the start pose and goal pose. Note: It is best to ensure that both poses have good expandability in advance: the start pose should be able to move forward normally (left front, right front, or straight ahead) for a limited arc length without collision, and the goal pose should be able to move backward normally (left rear, right rear, or straight rear) for a limited arc length without collision. Start and goal poses without expandability may lead to planning failure or excessive resource consumption.

// Input a binary obstacle map with only 0 and 255 pixel values.
// Optionally, input multiple potential goal poses.
// Return: The planning result, whether the start is trapped, the goal is trapped, or success. If multiple potential goal poses are provided, the one that is hit will be returned. The planned path is also returned.
// Additionally, it can return which segment of the path contains narrow passages (a path point is considered a narrow passage if there are obstacles within 20 cm on both the left and right sides of the robot at that pose).



