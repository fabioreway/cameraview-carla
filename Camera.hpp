/** @file
    @brief Definition of Class Camera
    @author Fabio Reway
*/
	
#ifndef CAMERA_H
#define CAMERA_H
 
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <chrono>
#include <malloc.h>
#include <new>

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <boost/lockfree/spsc_queue.hpp>

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;
using namespace cv;

#define EXPECT_TRUE(pred) if (!(pred)) { throw std::runtime_error(#pred); }


class Camera {
private:
    // Camera ID
    int m_id;

    // Carla related attributes for the virtual camera
    carla::client::ActorBlueprint m_blueprint;
    boost::shared_ptr<carla::client::Sensor> m_cam_ptr;
    carla::traffic_manager::ActorPtr m_vehicle_actor;

    // Camera proprierties
    carla::geom::Transform m_geo;     // mounting position
    int m_image_size_x; 			  // camera resolution	
    int m_image_size_y;			

    // OpenCV image matrix
    cv::Mat m_last_frame;
    boost::lockfree::spsc_queue<cv::Mat, boost::lockfree::capacity<200>>* m_imgMat;

public:
    Camera(carla::client::ActorBlueprint cam_bp, carla::traffic_manager::ActorPtr vehicle_actor);
    Camera(carla::geom::Transform geometry, carla::client::ActorBlueprint cam_bp, carla::traffic_manager::ActorPtr vehicle_actor);

    // Set functions
    void setGeometry(carla::geom::Transform geometry);
    void setBlueprint(carla::client::ActorBlueprint blueprint);
    void spawnCamera(carla::client::World &world);
    void registerCallbackFnc();

    // Get functions

/** 
    @brief Get camera id from object instance
    @return m_id - camera id
*/
    int getId() { return m_id; };

/** 
    @brief Get camera blueprint from carla
    @return m_blueprint - camera blueprint
*/
    carla::client::ActorBlueprint getBlueprint() { return m_blueprint; };

/** 
    @brief Get camera pointer
    @return m_cam_ptr - camera pointer 
*/
    boost::shared_ptr<carla::client::Sensor> getCameraPtr() { return m_cam_ptr; };

/** 
    @brief Get vehicle actor from CARLA
    @return m_vehicle_actor - camera pointer 
*/
    carla::traffic_manager::ActorPtr getVehicleActor() { return m_vehicle_actor; };

/** 
    @brief Get camera mounting position
    @return m_geo - camera mounting position
*/
    carla::geom::Transform getGeometry() { return m_geo; };

/** 
    @brief Get opencv image matrix
    @return m_imgMat - image matrix
*/
    boost::lockfree::spsc_queue<cv::Mat, boost::lockfree::capacity<200>>* getImgMat() { return m_imgMat; };

/** 
    @brief Get camera horizontal resolution
    @return m_image_size_x - horizontal resolution
*/
    int getResX() { return m_image_size_x; };

/** 
    @brief Get camera vertical resolution
    @return m_image_size_x - vertical resolution
*/
    int getResY() { return m_image_size_y; };

};

/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
    EXPECT_TRUE(range.size() > 0u);
    std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
    return range[dist(std::forward<RNG>(generator))];
}

#endif
