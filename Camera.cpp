/** @file
    @brief Implementation of Class Camera
	@author Fabio Reway
*/
	
#include "Camera.hpp"
  
/**
    @brief Camera Default Constructor (default mount position)
    @param cam_bp 			Camera blueprint
    @param vehicle_actor 	Vehicle actor where camera will be spawned
    @param m_id 			Camera ID 
    @param m_blueprint 		Camera blueprint
    @param m_vehicle_actor 	Vehicle Actor instance
    @param m_imgMat 		Image Data
*/
Camera::Camera(carla::client::ActorBlueprint cam_bp, carla::traffic_manager::ActorPtr vehicle_actor) 
    : m_id(-1), m_blueprint(cam_bp), m_vehicle_actor(vehicle_actor), m_imgMat(nullptr)
{
    std::cout << "> Default camera created" << std::endl;

    // Initialize with default camera mount position
    m_geo = cg::Transform{
        cg::Location{2.0f,  0.0f, 1.4f},   // x, y, z.
        cg::Rotation{-0.73f, 0.46f, -0.22f}}; // pitch, yaw, roll. 

    // Get image resolution 
    m_image_size_x = cam_bp.GetAttribute("image_size_x").As<int>();
    m_image_size_y = cam_bp.GetAttribute("image_size_y").As<int>();
}


/**
    @brief Camera Constructor with user-specified mounting position
    @param geo 				Camera mounting position
    @param cam_bp 			Camera blueprint
    @param vehicle_actor	Vehicle actor where camera will be spawned
    @param m_id 			Camera ID 
    @param m_blueprint 		Camera blueprint
    @param m_vehicle_actor 	Vehicle actor instance
    @param m_imgMat 		Image data
*/
Camera::Camera(carla::geom::Transform geo, carla::client::ActorBlueprint cam_bp, carla::traffic_manager::ActorPtr vehicle_actor) 
    : m_id(-1), m_blueprint(cam_bp), m_vehicle_actor(vehicle_actor), m_imgMat(nullptr)
{

    // Set mounting position
    setGeometry(geo);

    // Get image resolution
    m_image_size_x = cam_bp.GetAttribute("image_size_x").As<int>();
    m_image_size_y = cam_bp.GetAttribute("image_size_y").As<int>();
}

/** 
    @brief Set Camera Mounting Position
    @param geo Camera mounting position
*/
void Camera::setGeometry(carla::geom::Transform geo) {
    m_geo = geo;
}

/** 
    @brief Set Camera Blueprint
    @param blueprint Camera blueprint
*/
void Camera::setBlueprint(carla::client::ActorBlueprint blueprint) {
    m_blueprint = blueprint;    
}

/** 
    @brief Spawn camera to the ego vehicle
    @param world Carla World Instance
*/
void Camera::spawnCamera(carla::client::World &world) {
    // Get cam actor
    auto cam_actor = world.SpawnActor(m_blueprint, m_geo, m_vehicle_actor.get());
    m_cam_ptr = boost::static_pointer_cast<cc::Sensor>(cam_actor);
    m_id = m_cam_ptr->GetId();
    
    // Allocate que
    m_imgMat = new boost::lockfree::spsc_queue<cv::Mat, boost::lockfree::capacity<200>>;
}

/** 
    @brief Register camera callback function
*/
void Camera::registerCallbackFnc() {
    m_cam_ptr->Listen([this](auto data) {
        auto image = boost::static_pointer_cast<csd::Image>(data);
        EXPECT_TRUE(image != nullptr);
        Mat bgraMat(m_image_size_y, m_image_size_x, CV_8UC4, image->data());
        m_imgMat->push(bgraMat);
        // std::cout << ">> Callback | Camera-ID: " << m_id << std::endl;
    });
}

