//
// Created by Or Salmon on 15/08/18.
//

#ifndef ORB_SLAM2_GTSAM_TRANSFORMER_H
#define ORB_SLAM2_GTSAM_TRANSFORMER_H

#include <mutex>
#include <set>
#include <algorithm>
#include <queue>

#include "Converter.h"

// #include "System.h"

// For keys representation
#include <gtsam/inference/Symbol.h>

// For keyframes pose
#include <gtsam/geometry/StereoCamera.h>

// For between factors
#include <gtsam/slam/BetweenFactor.h>

// For landmarks position
#include <gtsam/geometry/Point3.h>

// For first keyframe pose
//#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/PriorFactor.h>

//// Mono ////
// For landmarks coordinates
#include <gtsam/geometry/Point2.h>
// For factors between keyframe and landmarks
#include <gtsam/slam/ProjectionFactor.h>

//// Stereo ////
// For landmarks coordinates
#include <gtsam/geometry/StereoPoint2.h>
// For factors between keyframe and landmarks
#include <gtsam/slam/StereoFactor.h>

// Factor Container
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
// Values Container
#include <gtsam/nonlinear/Values.h>

// Serialization
#include <gtsam/base/serialization.h>

// logger
#include <spdlog/spdlog.h>

namespace ORB_SLAM2 {
class KeyFrame;
class MapPoint;


class GtsamTransformer {
  friend class Optimizer;

  enum FactorType {
    PRIOR,
    BETWEEN,
    MONO,
    STEREO
  };
 public:
    struct additional_params_from_wrapper
    {
        char robot_id;
        std::string robot_name;
        short mpc_trigger;
        gtsam::Pose3 init_pose_rob;
        gtsam::Pose3 sensor_to_body_temp; // sensor to body transformation
        gtsam::noiseModel::Diagonal::shared_ptr between_factors_prior_;
        // future parameters
    } p_wrapper; //additional_params_from_wrapper from_wrapper;

  // GTSAM Transformer Update Type
  enum UpdateType{
    BATCH=0,
    INCREMENTAL=1
  };

 public:
    GtsamTransformer();
    GtsamTransformer(struct additional_params_from_wrapper& p);

    typedef std::queue<std::tuple<bool,
            bool,
            std::string,
            std::vector<size_t>,
            const gtsam::KeyList,
            const gtsam::KeyList,
            std::string,
            std::tuple<std::string, double, std::string>,
            gtsam::KeyVector,                      //added keyvectors for removed factors
            gtsam::KeyVector> > dataQueueTuple;    //added keyvectors for removed factors

    dataQueueTuple ready_data_queue_;

    typedef std::tuple<bool,
            boost::optional<bool>,
            boost::optional<std::string>,
            boost::optional<std::vector<size_t>>,
            boost::optional<const gtsam::KeyList>,
            boost::optional<const gtsam::KeyList>,
            boost::optional<std::string>,
            boost::optional<std::tuple<std::string, double, std::string>>,
            boost::optional<gtsam::KeyVector>,                     //added keyvectors for removed factors
            boost::optional<gtsam::KeyVector>> returnedTuple;      //added keyvectors for removed factors

    /**
   * Returns tuple contains:
   * 1. Boolean indicates if there is a new data or not
   * 2. Optional Boolean indicates if the data is incremental update
   * 3. Optional string contains graph of the added factors since the last call to the function (serialized)
   * 4. Optional vector contains the indices of the factors had removed since the last call to the function
   * 5. Optional GTSAM KeyList contains the keys of the added states (keyframes/landmarks) since the last call to the function
   * 6. Optional GTSAM KeyList contains the keys of the removed states (keyframes/landmarks) since the last call to the function
   * 7. Optional GTSAM Values object contains the values of entire graph (serialized)
   * 8. Optional tuple of the most recent keyframe symbol (serialized), its timestamp, and its Pose3 (serialized)
     *  9. Optional string contains graph of the removed factors first key  (serialized)
     * 10. Optional string contains graph of the removed factors second key  (serialized)
   **/

//  std::tuple<bool,
//             boost::optional<bool>,
//             boost::optional<std::string>,
//             boost::optional<std::vector<size_t>>,
//             boost::optional<const gtsam::KeyList>,
//             boost::optional<const gtsam::KeyList>,
//             boost::optional<std::string>,
//             boost::optional<std::tuple<std::string, double, std::string> >> checkForNewData();



  returnedTuple checkForNewData();


  void setUpdateType(const UpdateType update_type);

 protected:
  void transformGraphToGtsam(const std::vector<KeyFrame *> &vpKFs, const std::vector<MapPoint *> &vpMP);

 private:
  bool start();
  void finish();
  void updateKeyFrame(KeyFrame *pKF, bool add_between_factor = false);
  void updateKeyFrameBetween(KeyFrame *pKF, bool add_between_factor = false);
  void updateLandmark(MapPoint *pMP);
  void updateObservations(MapPoint *pMP, const std::map<KeyFrame *, size_t> &observations);
  void addMonoMeasurement(KeyFrame *pKF, MapPoint *pMP, Eigen::Matrix<double, 2, 1> &obs, const float inv_sigma_2);
  void addStereoMeasurement(KeyFrame *pKF, MapPoint *pMP, Eigen::Matrix<double, 3, 1> &obs, const float inv_sigma_2);
  void calculateDiffrencesBetweenValueSets();
  void calculateDiffrencesBetweenFactorSets();
  void exportKeysFromMap(std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> &map,
                         std::vector<std::pair<gtsam::Key, gtsam::Key>> &output);
  void exportValuesFromMap(std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> &map,
                           std::vector<std::pair<std::string, FactorType>> &output);
  std::string setToString(const std::set<gtsam::Key> &set) const;
  gtsam::NonlinearFactorGraph createFactorGraph(std::vector<std::pair<std::string, FactorType>> ser_factors_vec, bool is_incremental);
  gtsam::NonlinearFactorGraph createFactorGraph(std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> ser_factors_map,
                                                bool is_incremental);
  std::vector<size_t> createDeletedFactorsIndicesVec(std::vector<std::pair<gtsam::Key, gtsam::Key>> &del_factors);
  void createDeletedFactorsIndicesVec(std::vector<std::pair<gtsam::Key, gtsam::Key>> &del_factors, gtsam::KeyVector& k1, gtsam::KeyVector & k2);
  // Private implementation of std::set_difference
  std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> getDifferenceSet(std::map<std::pair<gtsam::Key, gtsam::Key>,
                                                                                                            std::pair<std::string,
                                                                                                                      FactorType>> &set_A,
                                                                                                   std::map<std::pair<gtsam::Key, gtsam::Key>,
                                                                                                            std::pair<std::string,
                                                                                                                      FactorType>> &set_B);
  gtsam::KeyList getDifferenceKeyList(const gtsam::KeyList &list_A, const gtsam::KeyList &list_B);





  //gtsam::NonlinearFactorGraph graph;
  gtsam::Values session_values_, last_session_values_, values_before_transf;
  gtsam::Cal3_S2Stereo::shared_ptr cam_params_stereo_;
  gtsam::Cal3_S2::shared_ptr cam_params_mono_;

  bool is_cam_params_initialized_ = false;
  std::vector<std::pair<std::string, FactorType>> add_factors_;
  std::vector<std::pair<gtsam::Key, gtsam::Key>> del_factors_;
  std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> session_factors_, last_session_factors_;
  gtsam::KeyList del_states_, add_states_;
  std::tuple<std::string, double, std::string> recent_kf_;
  std::map<std::pair<gtsam::Key, gtsam::Key>, size_t> factor_indecies_dict_;
  size_t current_index_ = 0;
  UpdateType update_type_;


  std::mutex mutex_;

  std::shared_ptr<spdlog::logger> logger_;
};
}

#endif //ORB_SLAM2_GTSAM_TRANSFORMER_H
