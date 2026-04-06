#include <iostream>
#include <random>

#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/World.hh>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/AxisAlignedBox.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/Physics.hh>
#include <ignition/gazebo/components/Collision.hh>

#include <ignition/msgs.hh>

#include <ignition/math/Pose3.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>

#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Utils.hh>
#include <ignition/math/Rand.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/physics/RequestEngine.hh>
#include <ignition/physics/Shape.hh>
#include <ignition/physics/FindFeatures.hh>

#include <sdf/Model.hh>



namespace ignition::gazebo 
{

  class RandomizedSpawner : public System,
  public ISystemConfigure,
  public ISystemPostUpdate

  {

      public: 
      double SampleX() { return distX(rng); }
      double SampleY() { return distY(rng); }

      virtual void Configure (const Entity & _entity, const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager & _ecm, EventManager & _eventMgr) override 
      {
	/* std::cout << "Starting Configuration! \n"; */

	this->worldEntity = ignition::gazebo::worldEntity(_ecm);
	this->environment = ignition::gazebo::World(worldEntity);


	/* if(this->worldEntity == ignition::gazebo::kNullEntity) { */
	/*   std::cout << "no world entity found\n"; */
	/* } */

	auto world_model = Model(_entity);
	/* std::cout << world_model.Name(_ecm) << std::endl; */
	this->worldName = world_model.Name(_ecm);

	/* this->node1_.Advertise<ignition::msgs::Pose>("/get_random_spawn", &RandomizedSpawner::OnGetSpawn, this); */
	this->node2_.Subscribe("/random_pose_req", &RandomizedSpawner::GetRequest, this);

	this->node1_.Subscribe("/world/" + this->worldName + "/model/laser0"+"/link/link/sensor/rplidar/scan", &RandomizedSpawner::OnRange, this); 
	this->node1_.Subscribe("/world/" + this->worldName + "/model/laser1"+"/link/link/sensor/rplidar/scan", &RandomizedSpawner::OnRange, this); 
	this->node1_.Subscribe("/world/" + this->worldName + "/model/laser2"+"/link/link/sensor/rplidar/scan", &RandomizedSpawner::OnRange, this); 
	this->node1_.Subscribe("/world/" + this->worldName + "/model/laser3"+"/link/link/sensor/rplidar/scan", &RandomizedSpawner::OnRange, this); 



	//Spawning Sensors
	


	pub = std::make_shared<ignition::transport::Node::Publisher>(
	    node2_.Advertise<ignition::msgs::Pose>("/random_pose_res"));


	
	this->sample_x = SampleX(), this->sample_y = SampleY();
	/* std::cout << "Configuration Finished! \n"; */

      }

      virtual void PostUpdate(const UpdateInfo & _info, 
	  const EntityComponentManager & _ecm) override
      {

	if(this->requestReceived) { 


	  if(this->model_itr < 4)  {

	    if(!this->scanRequested) {


	      if(!this->sensorSpawned) {

		/* std::cout << "init" << std::endl; */

		/* std::cout << delta_itr << " " << model_itr % 2 << std::endl; */

		SpawnOrMoveRaySensor(this->sample_x + this->delta[delta_itr], this->sample_y + this->delta[model_itr % 2], this->zHigh, model_itr);

		this->gotRange = false;
		this->ready[model_itr] = true;


		this->scanRequested = true;

	      }

	      else {	
		/* std::cout << "Moving model to " << this->sample_x + this->delta[delta_itr] << " " << this->sample_y + this->delta[model_itr % 2] << std::endl; */
		/* std::cout << "moving model " <<  this->model_x + this->sample_x << " "  << this->sample_y + this->sample_y << std::endl; */

		/* std::cout << delta_itr << " " << model_itr % 2 << std::endl; */

		ignition::msgs::Pose pose_msg;
		pose_msg.mutable_position()->set_x(this->model_x);
		pose_msg.mutable_position()->set_y(this->model_y);
		pose_msg.mutable_position()->set_z(0);
		pose_msg.set_name("laser"+std::to_string(model_itr));

		bool result; 
		ignition::msgs::Boolean rep; 
		this->node1_.Request("/world/" + this->worldName + "/set_pose", pose_msg, 1000, rep, result);

		//Step World
		ignition::msgs::WorldControl world_req;

		world_req.set_step(true);
		world_req.set_multi_step(1);
		/* world_req.set_pause(true); */

		bool world_result = false;

		node1_.Request("/world/" + this->worldName + "/control", world_req, 2000, rep, world_result);


		
		this->ready[model_itr] = true;
		this->gotRange = false;
		/* this->node1_.Subscribe("/world/" + this->worldName + "/model/laser"+std::to_string(model_itr)+"/link/link/sensor/rplidar/scan", &RandomizedSpawner::OnRange, this); */ 
		this->scanRequested = true;

	      }

	      if(model_itr >= 1) {
		delta_itr = 1;
	      }

	    }

	    if(this->scanRequested) {

	      if(this->gotRange) {

		/* std::cout << "Got Range: " << lastRange[model_itr] << std::endl; */

		if(this->lastRange[model_itr] < 10) this->invalidSpawn = true;

		/* node1_.Unsubscribe("/world/" + this->worldName + "/model/laser"+std::to_string(model_itr)+"/link/link/sensor/rplidar/scan"); */
		model_itr+=1;

		this->scanRequested = false;


	      }
	      else {

		ignition::msgs::WorldControl world_req;

		world_req.set_step(true);
		world_req.set_multi_step(1);
		/* world_req.set_pause(true); */

		bool world_result = false;
		ignition::msgs::Boolean rep; 

		node1_.Request("/world/" + this->worldName + "/control", world_req, 2000, rep, world_result);

		/* std::cout << "waiting for range\n"; */

		ignition::msgs::WorldControl world_unpause;

		world_unpause.set_pause(false);
		world_result = false;

		node1_.Request("/world/" + this->worldName + "/control", world_unpause, 2000, rep, world_result);


	      }

	    }


	  }


	  if(model_itr >= 4 && invalidSpawn) {


	    this->model_x = SampleX(), this->model_y = SampleY();
	    this->invalidSpawn = false;
	    this->model_itr = 0;
	    this->delta_itr = 0;
	    this->sensorSpawned = true;

	  }

	  if(model_itr >= 4 && !(this->invalidSpawn)) {

	    /* std::cout << this->model_x + this->sample_x << "  " << this->model_y + this->sample_y << std::endl; */
	    //publish the coordinates
	    ignition::msgs::Pose pose_msg;

	    pose_msg.mutable_position()->set_x(this->model_x + this->sample_x);
	    pose_msg.mutable_position()->set_y(this->model_y + this->sample_y);


	    pub->Publish(pose_msg);

	    //Step World
	    ignition::msgs::WorldControl world_req;

	    world_req.set_step(true);
	    world_req.set_multi_step(1);
	    /* world_req.set_pause(true); */

	    bool world_result = false;
	    ignition::msgs::Boolean rep; 

	    node1_.Request("/world/" + this->worldName + "/control", world_req, 2000, rep, world_result);
	    /* std::cout << "Removing Entities\n"; */

	    /* for(int i=0; i < model_itr; ++i) { */

	    /*   ignition::msgs::Entity removeReq; */
	    /*   ignition::msgs::Boolean removeRes; */
	    /*   bool removeOk = false; */

	    /*   removeReq.set_name("laser" + std::to_string(i)); */
	    /*   removeReq.set_type(ignition::msgs::Entity_Type_MODEL); */

	    /*   node1_.Request("/world/" + this->worldName + "/remove", removeReq, 2000, removeRes, removeOk); */

	    /* } */

	    this->model_x = SampleX(); this->model_y = SampleY();

	    this->requestReceived = false;
	    this->model_itr = 0;
	    this->delta_itr = 0;
	    this->sensorSpawned = true;
	    /* this->tryfindSpawn = 0; */



	  }



	}

	


      }



    private: void GetRequest(const ignition::msgs::Boolean &req_msg) 
	     {

	       this->requestReceived = req_msg.data();

	       /* if(this->requestReceived) std::cout << "Received a new request\n"; */


	     } 
    
    

    private: void SpawnOrMoveRaySensor(double x, double y, double z, int itr)
      {
	// Build a minimal SDF model with a single‑beam ray sensor
	
	std::ostringstream ss;
	ss << "<sdf version='1.8'>\n"
	   << "  <model name='laser" << itr << "'>\n"
	   << "    <static>true</static>\n"
	   << "    <link name='link'>\n"
	   << "      <pose>"
	   << std::fixed << std::setprecision(6)
	   << x << " " << y << " " << z << " 0 1.57 0</pose>\n"
	   << "      <sensor name='rplidar' type='gpu_ray'>\n"
	   << "        <ray>\n"
	   << "          <update_rate>62</update_rate>\n"
	   << "          <visualize>true</visualize>\n"
	   << "          <always_on>true</always_on>\n"
	   << "          <scan><horizontal>\n"
	   << "            <samples>1</samples>\n"
	   << "            <resolution>1</resolution>\n"
	   << "            <min_angle>0.0</min_angle>\n"
	   << "            <max_angle>0.0</max_angle>\n"
	   << "          </horizontal></scan>\n"
	   << "          <range><min>0.1</min><max>100</max><resolution>0.01</resolution></range>\n"
	   << "        </ray>\n"
	   << "    </sensor>\n"
	   << "    </link>\n"
	   << "  </model>\n"
	   << "</sdf>\n";

	std::string xml = ss.str();

	ignition::msgs::EntityFactory request;
	request.set_sdf(xml);


	ignition::msgs::Boolean response;
	bool result = false;

	/* std::cout << "Spawning" << std::endl; */

	std::string service = "/world/" + this->worldName + "/create";
	this->node1_.Request(service, request, 2000, response, result);  

	/* if(!result) std::cout << "Failed\n"; */
	/* std::cout << "Spawn done" << std::endl; */

      }

    private: void OnRange(const msgs::LaserScan &r)
      {

	auto range = r.ranges();

	if(this->ready[model_itr] == true) {

	  if(!range.empty() && std::isfinite(range[0])) { 
	    this->lastRange[model_itr] = range[0];
	  }
	  else { 
	    this->lastRange[model_itr] = std::numeric_limits<double>::infinity();
	  }

	  this->gotRange = true;
	  this->ready[model_itr] = false;
	}


      }

    private:
    bool spawned{false};


    double robotRadius = 0.345/2;
    double zHigh = 10.0;
    std::vector<double> delta = {-robotRadius, robotRadius};

    Entity worldEntity;
    World environment;

    std::string worldName;
    transport::Node node1_;
    transport::Node node2_;

    //Publisher
    std::shared_ptr<ignition::transport::Node::Publisher> pub;

    std::mt19937_64 rng{std::random_device{}()};
    bool scanRequested{false};
    
    std::mutex mtx;
    std::condition_variable cv;
    bool gotRange{false};

    int model_itr = 0;
    int delta_itr = 0;

    double lastRange[4];
    bool ready[4];

    double sample_x, sample_y;

    uint64_t entityIds[4];
    /* bool notRemoved = false; */


    int tryfindSpawn = 0;
    bool sensorSpawned{false};
    bool invalidSpawn{false};
    /* bool spawnFound{false}; */

    static std::uniform_real_distribution<double> distX{-5.0, 5.0};
    static std::uniform_real_distribution<double> distY{-5.0, 5.0};

    //Services
    const std::string ctrlService = "/world/" + this->worldName + "/control";
    const std::string poseService = "/world/" + this->worldName + "/set_pose";

    //Handing Requests
    bool requestReceived{false};

    //Saving link pose
    double model_x=0, model_y=0;



  };

}


IGNITION_ADD_PLUGIN(ignition::gazebo::RandomizedSpawner,
    ignition::gazebo::System, ignition::gazebo::RandomizedSpawner::ISystemConfigure, ignition::gazebo::RandomizedSpawner::ISystemPostUpdate)
