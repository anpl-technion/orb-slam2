//
// Created by Or Salmon on 15/08/18.
//

#include "GtsamTransformer.h"

#include "KeyFrame.h"
#include "MapPoint.h"
#include "Optimizer.h"

#include "GtsamSerializationHelper.h"
#include <string.h>




//#define DEBUG
//int counterMeasurmentTable = 1;
namespace ORB_SLAM2 {
    GtsamTransformer::GtsamTransformer() {
        logger_ = spdlog::rotating_logger_st("GtsamTransformer",
                                             "GtsamTransformer.log",
                                             1048576 * 50,
                                             3);
#ifdef DEBUG
        logger_->set_level(spdlog::level::debug);
#else
        logger_->set_level(spdlog::level::info);
#endif
        logger_->info("CTOR - GtsamTransformer instance created");
        //between_factors_prior_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-1, 1e-1, 1e-1, 0.1, 0.1, 0.1)); // yaw(rad) , pitch(rad), roll(rad) ,x,y,z, Elad,Andrej


        // Transformation from optical frame to robot frame, for now independent from ROS infrastructure
        //gtsam::Quaternion quat(0.5, -0.5, 0.5, -0.5);
//        gtsam::Point3 point(0.21, -0.06, 0.17);//[m]
//        sensor_to_body_temp = gtsam::Pose3(quat, point);
//
//        init_pose_robot = gtsam::Pose3(gtsam::Quaternion(0.707,0,0,0.707), gtsam::Point3(2,-6,0));
    }


    GtsamTransformer::GtsamTransformer(struct additional_params_from_wrapper& p) {
        logger_ = spdlog::rotating_logger_st("GtsamTransformer",
                                             "GtsamTransformer.log",
                                             1048576 * 50,
                                             3);
        p_wrapper = p;
        std::cout << "Data from orbwrapper = " << p_wrapper.robot_name << std::endl;

#ifdef DEBUG
        logger_->set_level(spdlog::level::debug);
#else
        logger_->set_level(spdlog::level::info);
#endif
        logger_->info("CTOR - GtsamTransformer instance created");
    }

    int MeasurmentTable[50000][3] = {-1};
    void GtsamTransformer::addMonoMeasurement(ORB_SLAM2::KeyFrame *pKF,
                                              ORB_SLAM2::MapPoint *pMP,
                                              Eigen::Matrix<double, 2, 1> &obs,
                                              const float inv_sigma_2) {
        logger_->debug("addMonoMeasurement - pKF->mnId: {}, pMP->mnId: {}", pKF->mnId, pMP->mnId);
        if (!is_cam_params_initialized_) {
            std::cout << "addMonoMeasurement - camera params has not been initialized!" << std::endl;
            exit(-2);
        }

        // Create both symbols
        gtsam::Symbol keyframe_sym(p_wrapper.robot_id , pKF->mnId);
        gtsam::Symbol landmark_sym('l', pMP->mnId);

        // add table entry
        // pMP->mnI, S = 0, M = 1

        int tmp = pMP->mnId;
//        cout << "tmp = " << tmp;
        MeasurmentTable[tmp][0]++;
        MeasurmentTable[tmp][2] = pKF->mnId;
//        cout << "  , MeasurmentTable[tmp] = " << MeasurmentTable[tmp][0] << "  " << MeasurmentTable[tmp][1] << endl;


        // Create landmark observation
        gtsam::Point2 obs_gtsam(obs(0), obs(1));

        // Create factor graph
        gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
                factor(obs_gtsam,
                       gtsam::noiseModel::Diagonal::Variances(Eigen::Vector2d(1 / inv_sigma_2, 1 / inv_sigma_2)),
                       keyframe_sym.key(),
                       landmark_sym.key(),
                       cam_params_mono_, p_wrapper.sensor_to_body_temp);
        session_factors_[std::make_pair(keyframe_sym.key(), landmark_sym.key())] = std::make_pair(gtsam::serialize(factor), FactorType::MONO);

    }

    void GtsamTransformer::addStereoMeasurement(ORB_SLAM2::KeyFrame *pKF,
                                                ORB_SLAM2::MapPoint *pMP,
                                                Eigen::Matrix<double, 3, 1> &obs,
                                                const float inv_sigma_2) {
        logger_->debug("addStereoMeasurement - pKF->mnId: {}, pMP->mnId: {}", pKF->mnId, pMP->mnId);
        if (!is_cam_params_initialized_) {
            std::cout << "addStereoMeasurement - camera params has not been initialized!" << std::endl;
            exit(-2);
        }
        // Create both symbols
        gtsam::Symbol keyframe_sym(p_wrapper.robot_id, pKF->mnId);
        gtsam::Symbol landmark_sym('l', pMP->mnId);

        // add table entry
        // pMP->mnI, S = 1, M = 0
        int tmp = pMP->mnId;
//        cout << "tmp = " << tmp;
        MeasurmentTable[tmp][1]++;
        MeasurmentTable[tmp][2] = pKF->mnId;
//        cout << "  , MeasurmentTable[tmp] = " << MeasurmentTable[tmp][0] << "  " << MeasurmentTable[tmp][1] << endl;

        // Create landmark observation
        gtsam::StereoPoint2 obs_gtsam(obs(0), obs(2), obs(1));

        gtsam::Matrix m;
        m.transpose();

        // Create factor graph
        gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>
                factor(obs_gtsam,
                       gtsam::noiseModel::Diagonal::Variances(Eigen::Vector3d(1 / inv_sigma_2, 1 / inv_sigma_2, 1 / inv_sigma_2)),
                       keyframe_sym.key(),
                       landmark_sym.key(),
                       cam_params_stereo_, p_wrapper.sensor_to_body_temp); // default:  boost::optional<POSE> body_P_sensor = boost::none => Identity, Andrej

        session_factors_[std::make_pair(keyframe_sym.key(), landmark_sym.key())] = std::make_pair(gtsam::serialize(factor), FactorType::STEREO);
    }

    GtsamTransformer::returnedTuple GtsamTransformer::checkForNewData() {

        if (ready_data_queue_.empty()) {
            logger_->debug("checkForNewData - there is no new data.");
            return std::make_tuple(false, boost::none, boost::none, boost::none, boost::none, boost::none, boost::none, boost::none,boost::none,boost::none);
        }
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (lock.owns_lock()) {
            logger_->info("checkForNewData - returning new optimized data. ready_data_queue.size: {}", ready_data_queue_.size());
            auto data = ready_data_queue_.front();
            ready_data_queue_.pop();
            //std::cout << "checkForNewData - returns " << (std::get<1>(data) ? "Incremental" : "Batch") << " update" << std::endl;
            return data;
        } else {
            logger_->error("checkForNewData - can't own mutex. returning false");
            return std::make_tuple(false, boost::none, boost::none, boost::none, boost::none, boost::none, boost::none, boost::none,boost::none,boost::none);
        }
    }

    bool GtsamTransformer::start() {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (lock.owns_lock()) {
            logger_->info("start - new recovering session.");
            add_states_.clear();
            del_states_.clear();
            session_values_.clear();
            values_before_transf.clear();

            add_factors_.clear();
            del_factors_.clear();
            session_factors_.clear();
            return true;
        } else {
            logger_->warn("start - can't own mutex. returns");
        }
        cout << "Inside start(): step 9" << endl;
        return false;
    }

    void GtsamTransformer::finish() {
        std::unique_lock<std::mutex> *lock;
        do {
            lock = new std::unique_lock<std::mutex>(mutex_, std::try_to_lock);
        } while (!lock->owns_lock());
        logger_->info("finish - ending recovering session. new_optimized_data is now available");
        logger_->info("finish - active states set size: {}", session_values_.size());
        logger_->info("finish - active factors vector size: {}", session_factors_.size());


        //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        //@@@@@@@@@@ del all values connected to just mono factors     @@@@@@@@@@@@@@@@@

        //ofstream myfile;
        //std::string tmp_s = "0";
//        if (counterMeasurmentTable > 9) {
//            tmp_s = "";
//        }
//        std::string path = "/usr/ANPLprefix/orb-slam2/DEBUG/MeasurmentTable_" + tmp_s + std::to_string(counterMeasurmentTable) + ".txt";
//        myfile.open(path);
        for(int count = 0; count < 50000; count ++){
            if ((MeasurmentTable[count][0] !=0) || (MeasurmentTable[count][1] !=0)) {
//                myfile << count <<  " " << MeasurmentTable[count][0] << " " << MeasurmentTable[count][1] << endl;
                if ((MeasurmentTable[count][0] > 0) && (MeasurmentTable[count][1] == 0)){
//                    myfile << count <<  " " << MeasurmentTable[count][0] << " " << MeasurmentTable[count][1] << endl;
                    gtsam::Symbol landmark_sym('l', count);
                    if (session_values_.exists(landmark_sym)) {
                        session_values_.erase(landmark_sym);
                    }
                }
            }
        }
//        myfile.close();
        memset(MeasurmentTable, 0, 50000*3*(sizeof(int)));
//        tmp_s.clear();
//        counterMeasurmentTable++;

        //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

        gtsam::KeyVector key_del_fac_first,key_del_fac_second; // List of keys for deleted FACTORS
        createDeletedFactorsIndicesVec(del_factors_,key_del_fac_first,key_del_fac_second);
        //std::string pathFGAF = "/usr/ANPLprefix/orb-slam2/FG_AF.txt";
        if (update_type_ == INCREMENTAL) {
            std::cout << "GT_transformer: send data in INCREMENTAL MODE\n";
            // Incremental update
            auto incremental_factor_graph = createFactorGraph(add_factors_, true);
            //gtsam::serializeToFile(incremental_factor_graph, pathFGAF);
            ready_data_queue_.emplace(true,
                                      true,
                                      gtsam::serialize(incremental_factor_graph),
                                      createDeletedFactorsIndicesVec(del_factors_),
                                      add_states_,
                                      del_states_,
                                      gtsam::serialize(session_values_),
                                      recent_kf_,
                                      key_del_fac_first,       // added to replace createDeletedFactorsIndicesVec
                                      key_del_fac_second);     // added to replace createDeletedFactorsIndicesVec
        } else if (update_type_ == BATCH) {
            std::cout << "GT_transformer: send data in BATCH MODE\n";
            // Batch update
            auto active_factor_graph = createFactorGraph(session_factors_, false);
            //gtsam::serializeToFile(active_factor_graph, pathFGAF);
            ready_data_queue_.emplace(true,
                                      false,
                                      gtsam::serialize(active_factor_graph),
                                      createDeletedFactorsIndicesVec(del_factors_), // this dummy function
                                      add_states_,
                                      del_states_,
                                      gtsam::serialize(session_values_),
                                      recent_kf_,
                                      key_del_fac_first,       // added to replace createDeletedFactorsIndicesVec
                                      key_del_fac_second);     // added to replace createDeletedFactorsIndicesVec
        }
        logger_->info("finish - ready_data_queue.size: {}", ready_data_queue_.size());
        std::cout << "finish - ready_data_queue.size = " << ready_data_queue_.size() << std::endl;
        


//        std::cout << "finish - session_factors.size: " << session_factors_.size() << " last_session_factors.size: " << last_session_factors_.size()
//                  << " add_factors.size: " << add_factors_.size()
//                  << " del_factors.size: " << del_factors_.size() << " add_states.size: " << add_states_.size() << " del_states.size: "
//                  << del_states_.size() << " values.size: " << session_values_.size() << " last_values.size: " << last_session_values_.size() << std::endl;



        last_session_values_ = session_values_;
        last_session_factors_ = session_factors_;

        delete lock;

    }

    void GtsamTransformer::exportKeysFromMap(std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> &map,
                                             std::vector<std::pair<gtsam::Key, gtsam::Key>> &output) {
        for (const auto &it: map) {
            output.push_back(it.first);
        }
    }

    void GtsamTransformer::exportValuesFromMap(std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> &map,
                                               std::vector<std::pair<std::string, FactorType>> &output) {
        for (const auto &it: map) {
            output.push_back(it.second);
        }
    }

    std::string GtsamTransformer::setToString(const std::set<gtsam::Key> &set) const {
        std::stringstream ss;
        for (const auto &it: set)
            ss << it << " ";
        return ss.str();
    }

    gtsam::NonlinearFactorGraph GtsamTransformer::createFactorGraph(std::vector<std::pair<std::string, FactorType>> ser_factors_vec,
                                                                    bool is_incremental) {
        // In use only in batch mode (not incremental)

        std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> new_active_factors;

        bool useMonoFactors = false; // Flag for de/enabling Monocular factors , Elad

        if (!is_incremental) {
            current_index_ = 0;
            factor_indecies_dict_.clear();
        }
        gtsam::NonlinearFactorGraph graph;
        for (const auto &it: ser_factors_vec) {
            switch (it.second) {
                case FactorType::PRIOR: {
                    gtsam::PriorFactor<gtsam::Pose3> prior_factor;
                    gtsam::deserialize(it.first, prior_factor);
                    graph.push_back(prior_factor);
                    factor_indecies_dict_[std::make_pair(prior_factor.keys()[0], prior_factor.keys()[1])] = current_index_++;
                    break;
                }
                case FactorType::BETWEEN: {
                    gtsam::BetweenFactor<gtsam::Pose3> between_factor;
                    gtsam::deserialize(it.first, between_factor);
                    if (!is_incremental && (del_factors_.size() > 0 || del_states_.size() > 0)) {
                        gtsam::Symbol first_sym(between_factor.keys().at(0));
                        gtsam::Symbol second_sym(between_factor.keys().at(1));
                        if ((std::find(del_factors_.begin(), del_factors_.end(), std::make_pair(first_sym.key(), second_sym.key())) != del_factors_.end())
                            || (std::find(del_states_.begin(), del_states_.end(), first_sym.key()) != del_states_.end())
                            || (std::find(del_states_.begin(), del_states_.end(), second_sym.key()) != del_states_.end())) {
                            break;
                        } else {
                            new_active_factors[std::make_pair(first_sym.key(), second_sym.key())] = it;
                        }
                    }
                    graph.push_back(between_factor);
                    factor_indecies_dict_[std::make_pair(between_factor.keys()[0], between_factor.keys()[1])] = current_index_++;
                    break;
                }
                case FactorType::MONO: {
                    if(useMonoFactors) {
                        gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> mono_factor;
                        gtsam::deserialize(it.first, mono_factor);
                        if (!is_incremental && (del_factors_.size() > 0 || del_states_.size() > 0)) {
                            gtsam::Symbol first_sym(mono_factor.keys().at(0));
                            gtsam::Symbol second_sym(mono_factor.keys().at(1));
                            if ((std::find(del_factors_.begin(), del_factors_.end(),
                                           std::make_pair(first_sym.key(), second_sym.key())) != del_factors_.end())
                                || (std::find(del_states_.begin(), del_states_.end(), first_sym.key()) !=
                                    del_states_.end())
                                || (std::find(del_states_.begin(), del_states_.end(), second_sym.key()) !=
                                    del_states_.end())) {
                                break;
                            } else {
                                new_active_factors[std::make_pair(first_sym.key(), second_sym.key())] = it;
                            }
                        }
                        graph.push_back(mono_factor);
                        factor_indecies_dict_[std::make_pair(mono_factor.keys()[0],
                                                             mono_factor.keys()[1])] = current_index_++;
                    }
                    break;
                }
                case FactorType::STEREO: {
                    gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> stereo_factor;
                    gtsam::deserialize(it.first, stereo_factor);
                    if (!is_incremental && (del_factors_.size() > 0 || del_states_.size() > 0)) {
                        gtsam::Symbol first_sym(stereo_factor.keys().at(0));
                        gtsam::Symbol second_sym(stereo_factor.keys().at(1));
                        if ((std::find(del_factors_.begin(), del_factors_.end(), std::make_pair(first_sym.key(), second_sym.key())) != del_factors_.end())
                            || (std::find(del_states_.begin(), del_states_.end(), first_sym.key()) != del_states_.end())
                            || (std::find(del_states_.begin(), del_states_.end(), second_sym.key()) != del_states_.end())) {
                            break;
                        } else {
                            new_active_factors[std::make_pair(first_sym.key(), second_sym.key())] = it;
                        }
                    }
                    graph.push_back(stereo_factor);
                    factor_indecies_dict_[std::make_pair(stereo_factor.keys()[0], stereo_factor.keys()[1])] = current_index_++;
                    break;
                }
            }
        }
        //std::cout << "createFactorGraph - size: " << graph.size() << std::endl;
        if (!is_incremental) {
            session_factors_ = new_active_factors;
            //cout << "new_active_factors.size() = " << new_active_factors.size() << endl;
            for (const auto &it: del_states_) {
                if (session_values_.find(it) != session_values_.end()) {
                    session_values_.erase(it);
                }
            }
        }
        return graph;
    }

    gtsam::NonlinearFactorGraph GtsamTransformer::createFactorGraph(map<pair<gtsam::Key, gtsam::Key>,
            pair<string, ORB_SLAM2::GtsamTransformer::FactorType>> ser_factors_map,
                                                                    bool is_incremental) {
        std::vector<std::pair<std::string, FactorType>> ser_factors_vec;
        for (const auto &it: ser_factors_map)
            ser_factors_vec.push_back(it.second);

        return createFactorGraph(ser_factors_vec, is_incremental);
    }

    void GtsamTransformer::createDeletedFactorsIndicesVec(std::vector<std::pair<gtsam::Key, gtsam::Key>> &del_factors, gtsam::KeyVector& k1, gtsam::KeyVector& k2) {
        for (const auto &it: del_factors) {
            k1.push_back(it.first);
            k2.push_back(it.second);
        }

    }

    std::vector<size_t> GtsamTransformer::createDeletedFactorsIndicesVec(std::vector<std::pair<gtsam::Key, gtsam::Key>> &del_factors) {
        std::vector<size_t> deleted_factors_indecies;
        /*for (const auto &it: del_factors) {
            auto dict_it = factor_indecies_dict_.find(it);
            if (dict_it != factor_indecies_dict_.end()) {
                deleted_factors_indecies.push_back(dict_it->second);

                gtsam::Symbol key1(it.first);
                gtsam::Symbol key2(it.second);
                std::cout << "createDeletedFactorsIndicesVec - " << key1.chr() << key1.index() << "-" << key2.chr() << key2.index() << " index: "
                          << dict_it->second << std::endl;
            }
        }*/

        return deleted_factors_indecies;
    }

    map<pair<gtsam::Key, gtsam::Key>, pair<string, GtsamTransformer::FactorType>> GtsamTransformer::getDifferenceSet(map<pair<gtsam::Key, gtsam::Key>,
            pair<string,
                    ORB_SLAM2::GtsamTransformer::FactorType>> &set_A,
                                                                                                                     map<pair<gtsam::Key, gtsam::Key>,
                                                                                                                             pair<string,
                                                                                                                                     ORB_SLAM2::GtsamTransformer::FactorType>> &set_B) {
        map<pair<gtsam::Key, gtsam::Key>, pair<string, GtsamTransformer::FactorType>> diff_set;
        for (const auto &it_A: set_A) {
            if (set_B.find(it_A.first) == set_B.end()) {
                diff_set.insert(it_A);
            }
        }
        return diff_set;
    }

    void GtsamTransformer::transformGraphToGtsam(const vector<ORB_SLAM2::KeyFrame *> &vpKFs, const vector<ORB_SLAM2::MapPoint *> &vpMP) {

        if (!start())
                return;
//        ofstream myfile;
//        std::string pathAF = "/usr/ANPLprefix/orb-slam2/afterKey.txt";

            for (const auto &pKF: vpKFs) {
                if (pKF->isBad())
                    continue;
                updateKeyFrame(pKF,
                               true); // Elad: bool condition for creating between fac using last opt values (true- use, false- guess what)
            }
            for (const auto &pKF: vpKFs) {
                if (pKF->isBad())
                    continue;
                updateKeyFrameBetween(pKF, true);

            }

            //gtsam::serializeToFile(session_values_, pathAF);
            //gtsam::serializeToFile(values_before_transf, pathBF);

            for (const auto &pMP: vpMP) {
                if (pMP->isBad())
                    continue;
                updateLandmark(pMP);
                const std::map<KeyFrame *, size_t> observations = pMP->GetObservations();
                updateObservations(pMP, observations);
            }

            calculateDiffrencesBetweenValueSets();
            calculateDiffrencesBetweenFactorSets();

            // loop over all landmarks in sesssion_vals
            // for each, chaeck check factors
            // if there is a lanmdark with only mono factors, remove these factors, and the landmark

            finish();
            //gtsam::serializeToFile(session_values_, pathLAF);
            //gtsam::serializeToFile(values_before_transf, pathLBF);


    }
    void GtsamTransformer::updateKeyFrame(ORB_SLAM2::KeyFrame *pKF, bool add_between_factor) {
        // Create keyframe symbol
        gtsam::Symbol sym(p_wrapper.robot_id, pKF->mnId);

        // Create camera parameters
        if (!is_cam_params_initialized_) {
            cam_params_stereo_.reset(new gtsam::Cal3_S2Stereo(pKF->fx, pKF->fy, 0.0, pKF->cx, pKF->cy, pKF->mb));
            cam_params_mono_.reset(new gtsam::Cal3_S2(cam_params_stereo_->calibration()));
            is_cam_params_initialized_ = true;

            // cout << "INIT POSE ROBOT: <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;
            // cout << p_wrapper.init_pose_rob << endl;
            // cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< " << endl;

        }


        // Create pose
        cv::Mat T_cv = pKF->GetPose();
        Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> T_gtsam(T_cv.ptr<float>(), T_cv.rows, T_cv.cols);
        gtsam::Pose3 left_cam_pose(T_gtsam.cast<double>());

        // Camera frame
        left_cam_pose = left_cam_pose.inverse();
        // +++++ Before Inverse Points -- For text files ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
        values_before_transf.insert(sym.key(), left_cam_pose);
        // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//


        left_cam_pose = p_wrapper.init_pose_rob.compose(p_wrapper.sensor_to_body_temp.compose(left_cam_pose)); // pose of the camera in the world frame
        //gtsam::StereoCamera stereo_cam(left_cam_pose, cam_params_stereo_);
        gtsam::Pose3 robot_pose = left_cam_pose.compose(p_wrapper.sensor_to_body_temp.inverse());
        session_values_.insert(sym.key(), robot_pose); //stereo_cam.pose()

        // Adding prior factor for x0
        if (pKF->mnId == 0) {
            auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6));
            gtsam::PriorFactor<gtsam::Pose3> prior_factor(gtsam::Symbol(p_wrapper.robot_id, 0), robot_pose, prior_noise);
            session_factors_[std::make_pair(sym.key(), sym.key())] = std::make_pair(gtsam::serialize(prior_factor), FactorType::PRIOR);
            //cout << "transformGraphToGtsam: Adding prior factor for x0" << endl;
        }

        // Adding between factor
        if (add_between_factor) {
            if (pKF->mnId > 0) {
                gtsam::Symbol sym_before(p_wrapper.robot_id, pKF->mnId - 1);
                if (session_values_.exists(sym_before.key())) {
                    gtsam::Pose3 relative_pose = robot_pose.between(session_values_.at<gtsam::Pose3>(sym_before.key())); //.between(gtsam::Pose3());
                    gtsam::BetweenFactor<gtsam::Pose3> between_factor(sym_before, sym, relative_pose, p_wrapper.between_factors_prior_);
                    session_factors_[std::make_pair(sym_before.key(), sym.key())] = std::make_pair(gtsam::serialize(between_factor), FactorType::BETWEEN);

                }
            }
        }

        // Update most recent keyframe
        if ((pKF->mTimeStamp > std::get<1>(recent_kf_)) || (pKF->mnId == 0)) {
            recent_kf_ = std::make_tuple(gtsam::serialize(sym), pKF->mTimeStamp, gtsam::serialize(robot_pose));
        }
    }



    void GtsamTransformer::updateKeyFrameBetween(ORB_SLAM2::KeyFrame *pKF, bool add_between_factor) {
        // Create keyframe symbol
        gtsam::Symbol sym(p_wrapper.robot_id, pKF->mnId);

        // Create camera parameters
        if (!is_cam_params_initialized_) {
            cam_params_stereo_.reset(new gtsam::Cal3_S2Stereo(pKF->fx, pKF->fy, 0.0, pKF->cx, pKF->cy, pKF->mb));
            cam_params_mono_.reset(new gtsam::Cal3_S2(cam_params_stereo_->calibration()));
            is_cam_params_initialized_ = true;
        }

        // Create pose
        cv::Mat T_cv = pKF->GetPose();
        Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> T_gtsam(T_cv.ptr<float>(), T_cv.rows, T_cv.cols);
        gtsam::Pose3 left_cam_pose(T_gtsam.cast<double>());

        left_cam_pose = p_wrapper.init_pose_rob.compose(p_wrapper.sensor_to_body_temp.compose(left_cam_pose));
        gtsam::Pose3 robot_pose = left_cam_pose.compose(p_wrapper.sensor_to_body_temp.inverse());

        // Adding between factor
        if (add_between_factor) {
            if (pKF->mnId > 0) {
                gtsam::Symbol sym_before(p_wrapper.robot_id, pKF->mnId - 1);
                if (session_values_.exists(sym_before.key())) {
                    gtsam::Pose3 relative_pose = robot_pose.between(session_values_.at<gtsam::Pose3>(sym_before.key())); //.between(gtsam::Pose3());
                    gtsam::BetweenFactor<gtsam::Pose3> between_factor(sym_before, sym, relative_pose, p_wrapper.between_factors_prior_);
                    session_factors_[std::make_pair(sym_before.key(), sym.key())] = std::make_pair(gtsam::serialize(between_factor), FactorType::BETWEEN);

                }
            }
        }


    }



    void GtsamTransformer::updateLandmark(ORB_SLAM2::MapPoint *pMP) {
        // Create landmark symbol
        gtsam::Symbol sym('l', pMP->mnId);



        // Create landmark position
        cv::Mat p_cv = pMP->GetWorldPos();
        //gtsam::Point3 p_gtsam(p_cv.at<float>(0), p_cv.at<float>(1), p_cv.at<float>(2));
        gtsam::Vector3 t_vector(p_cv.at<float>(0), p_cv.at<float>(1), p_cv.at<float>(2));

        /*// +++++ Before Inverse Points -- For text files ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
        gtsam::Vector3 t_vector_BF(p_cv.at<float>(0), p_cv.at<float>(1), p_cv.at<float>(2));
        //t_vector_BF += p_wrapper.init_pose_rob.translation().vector() + p_wrapper.sensor_to_body_temp.translation().vector();
        gtsam::Point3 p_gtsam_BF(t_vector_BF.x(), t_vector_BF.y(), t_vector_BF.z());
        values_before_transf.insert(sym.key(), p_gtsam_BF);
        // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//*/

        t_vector = p_wrapper.init_pose_rob.rotation().matrix()*p_wrapper.sensor_to_body_temp.rotation().matrix()*t_vector;
        t_vector += p_wrapper.init_pose_rob.translation().vector() + p_wrapper.sensor_to_body_temp.translation().vector();

        gtsam::Point3 p_gtsam(t_vector.x(), t_vector.y(), t_vector.z());
        session_values_.insert(sym.key(), p_gtsam);

//        gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information(2*gtsam::eye(3));
//        session_factors_[std::make_pair(sym.key(), sym.key())] = std::make_pair(gtsam::serialize(gtsam::PriorFactor<gtsam::Point3>(sym,p_gtsam,model)), FactorType::PRIOR);

//        gtsam::Pose3 robot_pose;
//        auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 1e-6, 1e-6, 1e-6));
//        gtsam::PriorFactor<gtsam::Pose3> prior_factor(sym, robot_pose, prior_noise);
//        session_factors_[std::make_pair(sym.key(), sym.key())] = std::make_pair(gtsam::serialize(prior_factor), FactorType::PRIOR);
//        cout << "transformGraphToGtsam: Adding prior factor :" << endl;
//        sym.print();
    }

    void GtsamTransformer::updateObservations(MapPoint *pMP, const map<ORB_SLAM2::KeyFrame *, size_t> &observations) {
        for (const auto &mit: observations) {
            KeyFrame *pKFi = mit.first;
            if (pKFi->isBad())
                continue;
            const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit.second];
            // Monocular observation
            if (pKFi->mvuRight[mit.second] < 0) {
                Eigen::Matrix<double, 2, 1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;
                const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                addMonoMeasurement(pKFi, pMP, obs, invSigma2);
            } else // Stereo observation
            {
                Eigen::Matrix<double, 3, 1> obs;
                const float kp_ur = pKFi->mvuRight[mit.second];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
                const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                addStereoMeasurement(pKFi, pMP, obs, invSigma2);
            }
        }
    }

    void GtsamTransformer::calculateDiffrencesBetweenValueSets() {
        // Handle added states

        if (last_session_values_.empty()) {
            add_states_ = session_values_.keys();
        } else {
            add_states_ = getDifferenceKeyList(session_values_.keys(), last_session_values_.keys());
        }

        // Handle deleted states
        del_states_ = getDifferenceKeyList(last_session_values_.keys(), session_values_.keys());
    }

    void GtsamTransformer::calculateDiffrencesBetweenFactorSets() {
        // Handle added factors
        std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> add_factors_map;
        if (last_session_factors_.empty()) {
            add_factors_map = session_factors_;
            exportValuesFromMap(add_factors_map, add_factors_);
        } else {
            add_factors_map = getDifferenceSet(session_factors_, last_session_factors_);
            exportValuesFromMap(add_factors_map, add_factors_);
        }

        // Handle deleted factors
        std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>>
                del_factors_map = getDifferenceSet(last_session_factors_, session_factors_);
        exportKeysFromMap(del_factors_map, del_factors_);
    }

    gtsam::KeyList GtsamTransformer::getDifferenceKeyList(const gtsam::KeyList &list_A, const gtsam::KeyList &list_B) {
        gtsam::KeyList diff_list;
        for (const auto &it_A: list_A) {
            if (std::find(list_B.begin(), list_B.end(), it_A) == list_B.end()) {
                diff_list.push_back(it_A);
            }
        }
        return diff_list;
    }

    void GtsamTransformer::setUpdateType(const ORB_SLAM2::GtsamTransformer::UpdateType update_type) {
        update_type_ = update_type;
        if (update_type_ == BATCH) {
            logger_->info("setUpdateType - Batch");
        } else if (update_type_ == INCREMENTAL) {
            logger_->info("setUpdateType - Incremental");
        }
    }
}