/** 	@file
	@brief Main file
	@author Fabio Reway
*/

#include "Camera.hpp"
#include "cxxopts.hpp"
#include <omp.h>

// IPG Video Interface Box maximum resolution for 4 cameras
#define VIB_RES_X 7680
#define VIB_RES_Y 1232

/// Static functions
static volatile bool keep_running = true;
static void* userInput_thread(void*);
static void calcNumOfCams(cxxopts::ParseResult &result, int &max_horz, int &max_vert, int &num_cams);

/// Carla related functions
static carla::client::ActorBlueprint getRGBCamBlueprint(boost::shared_ptr<carla::client::BlueprintLibrary> &blueprint_library, cxxopts::ParseResult &result);
static std::vector<Camera> createCamera(int numCameras, carla::client::ActorBlueprint &cam_bp, carla::traffic_manager::ActorPtr &vehicle_actor);
static void spawnCameras(std::vector<Camera> &cam_vector, carla::client::World &world);
static void destroyAllCameras(std::vector<Camera> &cam_vector);


/** 
    @brief Main function that connects to carla simulation and manages the opencv window for each cam to display image data
*/
int main(int argc, const char *argv[]) {
    try {

        cxxopts::Options options("CARLA CameraView", "A simple code to deploy virtual cameras in a carla actor");

        // CLI commands for creating and spawning cameras
        options.add_options()
            ("s,server", "Carla Server", cxxopts::value<std::string>()->default_value("localhost"))
            ("p,port", "Port number", cxxopts::value<uint16_t>()->default_value("2000"))
            ("i,actor-id", "Actor ID", cxxopts::value<uint16_t>()->default_value("86"))
            ("n,num-cams", "Number of cameras", cxxopts::value<int>()->default_value("4"))
            ("m,max-cams", "Deploy maximum number of cameras", cxxopts::value<bool>()->default_value("false"))
            ("x,resx", "Resolution in X", cxxopts::value<std::string>()->default_value("1920"))
            ("y,resy", "Resolution in Y", cxxopts::value<std::string>()->default_value("1232"))
            ("f,fieldofview", "Camera Field of View", cxxopts::value<std::string>()->default_value("60"))
            ("h,help", "Print usage")
        ;

        auto result = options.parse(argc, argv);
        
        // CLI help 
        if (result.count("help")) {
            std::cout << options.help({""}) << std::endl;
            exit(0);
        }

        // Variables declaration
        pthread_t tId; // thread for waiting keyboard input
        uint v_actor_id;
        int num_cams=-1, max_horz=-1, max_vert=-1;
        std::vector<Camera> cam_vector;

        // Connect to CARLA Server
        std::string server = result["server"].as<std::string>();
        uint16_t port = result["port"].as<uint16_t>();

        auto client = cc::Client(server, port);
        client.SetTimeout(10s);

        // Get CARLA Blueprints
        auto world = client.GetWorld();
        auto blueprint_library = world.GetBlueprintLibrary();
        auto rgb_cam_bp = getRGBCamBlueprint(blueprint_library, result);

        std::cout << "Client API version : " << client.GetClientVersion() << '\n';
        std::cout << "Server API version : " << client.GetServerVersion() << '\n';

        // Define Vehicle Actor ID
        v_actor_id = result["actor-id"].as<uint16_t>();

        // Get Actor by ID
        auto vehicle_actor = world.GetActor(v_actor_id); 
        EXPECT_TRUE(vehicle_actor != nullptr);
        std::cout << "> Got actor! " << vehicle_actor->GetDisplayId() << std::endl;
        
        // Construct Camera Objects
        calcNumOfCams(result, max_horz, max_vert, num_cams);
        cam_vector = createCamera(num_cams, rgb_cam_bp, vehicle_actor);

        // Print number of to-be deployed cameras
        std::cout << "Num cams: " << num_cams << " Max Horz: " << max_horz << " Max Vert: " << max_vert << std::endl;

        // Spawn cameras
        spawnCameras(cam_vector, world);

        // Create thred to monitor user key input
        (void) pthread_create(&tId, 0, userInput_thread, 0);

        // Create window to display camera image according to id
        int cam_idx = 0;
        for (int j=0; (j < max_vert) && (cam_idx < num_cams); j++) {
            for (int i=0; (i < max_horz) && (cam_idx < num_cams); i++) {
                int pos_x = i * cam_vector.at(cam_idx).getResX();
                int pos_y = j * cam_vector.at(cam_idx).getResY();
                namedWindow("VIB " + std::to_string(cam_vector.at(cam_idx).getId()), WINDOW_AUTOSIZE);
                moveWindow("VIB " + std::to_string(cam_vector.at(cam_idx).getId()), +pos_x, +pos_y);
                cam_idx++;
            }
        }

        // Run infinite loop
        while(keep_running) {     
            #pragma omp for 
            for (uint i=0; i < cam_vector.size(); i++) {
                auto imgMat = cam_vector.at(i).getImgMat();
                cv::Mat img;
                if(imgMat->pop(img)) {
                    // std::cout << "ReadA: " << imgMat->read_available() << " WriteA: " << imgMat->write_available() << std::endl;
                    imshow("VIB " + std::to_string(cam_vector.at(i).getId()), img);
                    waitKey(1);       
                }
            }
        } 
        (void) pthread_join(tId, NULL);

        // Destroy cameras in simulation when application exists
        if(!keep_running){
            destroyAllCameras(cam_vector);  
        }

    } catch (const cc::TimeoutException &e) {
        std::cout << '\n' << e.what() << std::endl;
        return 1;
    } catch (const std::exception &e) {
    std::cout << "\nException: " << e.what() << std::endl;
        return 2;
    }
}

/** 
    @brief Monitor user input to stop main loop when 'q' is pressed
*/
static void* userInput_thread(void*) {
    while(keep_running) {
        if (std::cin.get() == 'q') {
            keep_running = false; //! desired user input 'q' received
        }
    }
    return (void*)keep_running;
}


/** 
    @brief Calculate maximum number of cameras based on the defined resolution 

    @param result		CLI parsed commands	
    @param max_horz		Max horizontal cam resolution
    @param max_vert 	Max vertical cam resolution
    @param num_cams 	Number of virtual cams to be created
*/
static void calcNumOfCams(cxxopts::ParseResult &result, int &max_horz, int &max_vert, int &num_cams) {
    bool max_cams = result["max-cams"].as<bool>();
    int cam_x_res = std::stoi(result["x"].as<std::string>());
    int cam_y_res = std::stoi(result["y"].as<std::string>());

    max_horz = VIB_RES_X / cam_x_res;
    max_vert = VIB_RES_Y / cam_y_res;
    
    if (max_cams)
        num_cams = (max_horz*max_vert);
    else
        num_cams = result["num-cams"].as<int>();
}

/** 
    @brief Get Blueprint of RGB camera 

    @param 	blueprint_library	Carla blueprint lib
    @param 	result				CLI parsed commands	
    @return blueprint_cam		Camera blueprint
*/
static carla::client::ActorBlueprint getRGBCamBlueprint(boost::shared_ptr<carla::client::BlueprintLibrary> &blueprint_library, cxxopts::ParseResult &result) {
    auto camera_bp = blueprint_library->Filter("sensor.camera.rgb");
    auto blueprint_cam = camera_bp->at(0);
    blueprint_cam.SetAttribute("image_size_x", result["x"].as<std::string>());
    blueprint_cam.SetAttribute("image_size_y", result["y"].as<std::string>());
    blueprint_cam.SetAttribute("fov", result["f"].as<std::string>());
    return blueprint_cam;
}

/** 
    @brief Create camera vector

    @param 	numCameras		Number of cameras to be created in simulation
    @param 	cam_bp			Camera blueprint
    @param 	vehicle_actor	Ego vehicle actor where cameras will be spawned
    @return cam_vector		Camera vector
*/
static std::vector<Camera> createCamera(int numCameras, carla::client::ActorBlueprint &cam_bp, carla::traffic_manager::ActorPtr &vehicle_actor) {
    std::vector<Camera> cam_vector;

		// If number of cams is equal 4, then create front, rear, left-side and right-side cams  
		if (numCameras==4) {
				carla::geom::Transform front = cg::Transform{cg::Location{2.0f,  0.0f, 1.4f}, cg::Rotation{-0.73f, 0.46f, -0.22f}}; 
				carla::geom::Transform rear = cg::Transform{cg::Location{-2.0f,  0.0f, 1.5f}, cg::Rotation{9.08f, 180.0f, -0.68f}}; 
				carla::geom::Transform left = cg::Transform{cg::Location{0.7f,  -0.75f, 1.5f}, cg::Rotation{3.4f, -93.5f, 0.7f}}; 
				carla::geom::Transform right = cg::Transform{cg::Location{0.7f,  0.75f, 1.5f}, cg::Rotation{1.45f, 90.0f, -0.4f}}; 

				Camera frontcam(front, cam_bp, vehicle_actor);
				Camera rearcam(rear, cam_bp, vehicle_actor);
				Camera leftcam(left, cam_bp, vehicle_actor);
				Camera rightcam(right, cam_bp, vehicle_actor);

				cam_vector.push_back(frontcam);        
				cam_vector.push_back(rearcam);
				cam_vector.push_back(leftcam);
				cam_vector.push_back(rightcam);
		} 
		// else creates front cams only
		else {
				for (int i=0; i<numCameras; i++) {
				    Camera cam(cam_bp, vehicle_actor);
				    cam_vector.push_back(cam);
				}
		}

    return cam_vector;
}

/** 
    @brief Spawn cameras in the ego vehicle

    @param 	cam_vector		Camera vector
    @param 	world			Carla world instance
*/
static void spawnCameras(std::vector<Camera> &cam_vector, carla::client::World &world) {
    for (Camera &c : cam_vector) {
        c.spawnCamera(world);
        std::cout << "> Camera spawned | " << "Cam-ID: " << c.getId() << " | Vehicle-ID: " << c.getVehicleActor()->GetDisplayId() << std::endl;
        
        c.registerCallbackFnc();
        std::cout << "> Callback registered!" << std::endl;
        std::cout << ">> ID: " << c.getId() << " | V-ID: " << c.getVehicleActor()->GetDisplayId() << std::endl;
    }
}

/** 
    @brief Destroy cameras

    @param cam_vector		Camera vector
*/
static void destroyAllCameras(std::vector<Camera> &cam_vector) {
    for (Camera &c : cam_vector) {
        c.getCameraPtr()->Destroy();
        std::cout << "> Camera ID " << c.getId() << " destroyed." << std::endl;
    }

}
