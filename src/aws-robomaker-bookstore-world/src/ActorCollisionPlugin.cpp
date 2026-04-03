#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{

/// ModelPlugin that moves a collision proxy model to track actor movement
class ActorCollisionPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    proxy_ = model;
    world_ = model->GetWorld();

    if (!sdf->HasElement("actor_name")) {
      gzerr << "[ActorCollisionPlugin] Missing <actor_name> element.\n";
      return;
    }
    actor_name_ = sdf->Get<std::string>("actor_name");

    // z offset so cylinder centre sits at actor waist height (half of 1.8 m)
    z_offset_ = sdf->HasElement("z_offset") ? sdf->Get<double>("z_offset") : 0.9;

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ActorCollisionPlugin::OnUpdate, this));
  }

private:
  void OnUpdate()
  {
    if (!actor_) {
      // Resolve actor lazily
      physics::BasePtr base = world_->BaseByName(actor_name_);
      if (!base) { return; }
      actor_ = boost::dynamic_pointer_cast<physics::Actor>(base);
      if (!actor_) { return; }
    }

    ignition::math::Pose3d actor_pose = actor_->WorldPose();
    ignition::math::Pose3d proxy_pose(
      actor_pose.Pos().X(),
      actor_pose.Pos().Y(),
      z_offset_,
      0, 0, 0);
    proxy_->SetWorldPose(proxy_pose);
  }

  physics::ModelPtr proxy_;
  physics::ActorPtr actor_;
  physics::WorldPtr world_;
  std::string actor_name_;
  double z_offset_{0.9};
  event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(ActorCollisionPlugin)

}  // namespace gazebo
