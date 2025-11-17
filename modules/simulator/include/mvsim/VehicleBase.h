/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <box2d/b2_body.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_polygon_shape.h>
#include <box2d/b2_world.h>
#include <mrpt/img/TColor.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPose2D.h>
#include <mvsim/ClassFactory.h>
#include <mvsim/ControllerBase.h>
#include <mvsim/FrictionModels/FrictionBase.h>
#include <mvsim/Sensors/SensorBase.h>
#include <mvsim/Simulable.h>
#include <mvsim/VisualObject.h>
#include <mvsim/Wheel.h>
#include <mvsim/basic_types.h>

#include <atomic>
#include <mutex>
#include <string>

#include "CsvLogger.h"

namespace mvsim
{
/** Virtual base class for each vehicle "actor" in the simulation.
 * Derived classes implements different dynamical models (Differential,
 * Ackermann,...)
 *
 *  \ingroup virtual_interfaces_module
 */
class VehicleBase : public VisualObject, public Simulable
{
   public:
	using Ptr = std::shared_ptr<VehicleBase>;

	/** Class factory: Creates a vehicle from XML description of type
	 * "<vehicle>...</vehicle>".  */
	static Ptr factory(World* parent, const rapidxml::xml_node<char>* xml_node);
	/// \overload
	static Ptr factory(World* parent, const std::string& xml_text);

	/** Register a new class of vehicles from XML description of type
	 * "<vehicle:class name='name'>...</vehicle:class>".  */
	static void register_vehicle_class(
		const World& parent, const rapidxml::xml_node<char>* xml_node);

	// ------- Interface with "World" ------
	virtual void simul_pre_timestep(const TSimulContext& context) override;
	virtual void simul_post_timestep(const TSimulContext& context) override;
	virtual void apply_force(
		const mrpt::math::TVector2D& force,
		const mrpt::math::TPoint2D& applyPoint = mrpt::math::TPoint2D(0, 0)) override;

	/** Create bodies, fixtures, etc. for the dynamical simulation. May be
	 * overrided by derived classes */
	virtual void create_multibody_system(b2World& world);

	/** Get (an approximation of) the max radius of the vehicle, from its point
	 * of reference (in meters) */
	virtual float getMaxVehicleRadius() const { return maxRadius_; }
	/** Get the overall vehicle mass, excluding wheels. */
	virtual double getChassisMass() const { return chassis_mass_; }
	b2Body* getBox2DChassisBody() { return b2dBody_; }
	mrpt::math::TPoint2D getChassisCenterOfMass() const
	{
		return chassis_com_;
	}  //!< In local coordinates (this excludes the mass of wheels)

	size_t getNumWheels() const { return wheels_info_.size(); }
	const Wheel& getWheelInfo(const size_t idx) const { return wheels_info_[idx]; }
	Wheel& getWheelInfo(const size_t idx) { return wheels_info_[idx]; }

	/** Current velocity of each wheel's center point (in local coords). Call
	 * with veh_vel_local=getVelocityLocal() for ground-truth.  */
	std::vector<mrpt::math::TVector2D> getWheelsVelocityLocal(
		const mrpt::math::TTwist2D& veh_vel_local) const;

	/** Gets the current estimation of odometry-based velocity as reconstructed
	 * solely from wheels spinning velocities and geometry.
	 * This is the input of any realistic low-level controller onboard.
	 * \sa getVelocityLocal() */
	virtual mrpt::math::TTwist2D getVelocityLocalOdoEstimate() const = 0;

	const TListSensors& getSensors() const { return sensors_; }
	TListSensors& getSensors() { return sensors_; }
	CSVLogger::Ptr getLoggerPtr(const std::size_t logger_index)
	{
		return loggers_.at(logger_index);
	}

	/** Get the 2D shape of the vehicle chassis, as set from the config file
	 * (only used for collision detection) */
	const mrpt::math::TPolygon2D& getChassisShape() const { return chassis_poly_; }

	/** Set the vehicle index in the World */
	void setVehicleIndex(size_t idx) { vehicle_index_ = idx; }
	/** Get the vehicle index in the World */
	size_t getVehicleIndex() const { return vehicle_index_; }
	void setRecording(bool record)
	{
		for (auto& logger : loggers_)
		{
			logger->setRecording(record);
		}
	}
	void clearLogs()
	{
		for (auto& logger : loggers_)
		{
			logger->clear();
		}
	}
	void newLogSession()
	{
		for (auto& logger : loggers_)
		{
			logger->newSession();
		}
	}

	virtual ControllerBaseInterface* getControllerInterface() = 0;

	void registerOnServer(mvsim::Client& c) override;

	b2Fixture* get_fixture_chassis() { return fixture_chassis_; }
	std::vector<b2Fixture*>& get_fixture_wheels() { return fixture_wheels_; }
	const b2Fixture* get_fixture_chassis() const { return fixture_chassis_; }
	const std::vector<b2Fixture*>& get_fixture_wheels() const { return fixture_wheels_; }

	void freeOpenGLResources() override
	{
		for (auto& sensor : sensors_)
		{
			sensor->freeOpenGLResources();
		}
	}
	void chassisAndWheelsVisible(bool visible);

	double chassisZMin() const { return chassis_z_min_; }
	double chassisZMax() const { return chassis_z_max_; }

	/** Gets the noisy, biased, odometry estimation.
	 */
	mrpt::poses::CPose2D getOdometry() const { return odometry_; }

	void resetOdometry()
	{
		odometry_ = mrpt::poses::CPose2D();
		odometry_pre_simul_pose_.reset();
	}

   protected:
	std::vector<CSVLogger::Ptr> loggers_;
	std::string log_path_;

	virtual void initLoggers();
	virtual void writeLogStrings();
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly) override;

   protected:
	// Protected ctor for class factory
	VehicleBase(World* parent, size_t nWheels);

	/** Parse node <dynamics>: The derived-class part of load_params_from_xml(),
	 * also called in factory(). Includes parsing the <controller></controller>
	 * block. */
	virtual void dynamics_load_params_from_xml(const rapidxml::xml_node<char>* xml_node) = 0;

	virtual std::vector<double> invoke_motor_controllers(const TSimulContext& context) = 0;

	virtual void invoke_motor_controllers_post_step([[maybe_unused]] const TSimulContext& context)
	{
	}

	VisualObject* meAsVisualObject() override { return this; }

	/** user-supplied index number: must be set/get'ed with setVehicleIndex()
	 * getVehicleIndex() (default=0) */
	size_t vehicle_index_ = 0;

	/** Instances of friction model for the vehicle-to-ground interaction, for each wheel */
	std::vector<FrictionBase::Ptr> frictions_;

	TListSensors sensors_;	//!< Sensors aboard

	// Chassis info:
	double chassis_mass_ = 15.0;
	mrpt::math::TPolygon2D chassis_poly_;

	/** Automatically computed from chassis_poly_ upon each change via
	 * updateMaxRadiusFromPoly() */
	double maxRadius_ = 0.1;

	double chassis_z_min_ = 0.05, chassis_z_max_ = 0.6;

	mrpt::img::TColor chassis_color_{0xff, 0x00, 0x00};

	/** center of mass. in local coordinates (this excludes the mass of wheels)
	 */
	mrpt::math::TPoint2D chassis_com_{0, 0};

	void updateMaxRadiusFromPoly();

	/** Noisy odometry parameters */
	struct OdometryNoise
	{
		// Initializes the multipliers with random factors near 1.0
		OdometryNoise();

		[[nodiscard]] mrpt::poses::CPose2D actualDeltaToNoisyOdo(
			const mrpt::poses::CPose2D& delta) const
		{
			return {
				delta.x() * x_multiplier, delta.y() * y_multiplier, delta.phi() * yaw_multiplier};
		}

		double x_multiplier;
		double y_multiplier;
		double yaw_multiplier;
	};
	OdometryNoise odometry_noise_;

	/** Wheels info.  The fixed size of this vector is set upon construction.
	 *  Derived classes must define the order of the wheels, e.g. [0]=rear left,
	 * etc.
	 */
	std::deque<Wheel> wheels_info_;

	// Box2D elements:
	b2Fixture* fixture_chassis_ = nullptr;	//!< Created at

	/** [0]:rear-left, etc. (depending on derived class). Size set at
	 * constructor. */
	std::vector<b2Fixture*> fixture_wheels_;

   private:
	// Called from internalGuiUpdate()
	void internal_internalGuiUpdate_sensors(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical);
	// Called from internalGuiUpdate()
	void internal_internalGuiUpdate_forces(mrpt::opengl::COpenGLScene& scene);

	mrpt::opengl::CSetOfObjects::Ptr glChassisViz_, glChassisPhysical_;
	std::vector<mrpt::opengl::CSetOfObjects::Ptr> glWheelsViz_, glWheelsPhysical_;
	mrpt::opengl::CSetOfLines::Ptr glForces_;
	mrpt::opengl::CSetOfLines::Ptr glMotorTorques_;
	std::atomic_bool glInit_ = false;

	std::vector<mrpt::math::TSegment3D> forceSegmentsForRendering_;
	std::vector<mrpt::math::TSegment3D> torqueSegmentsForRendering_;
	std::mutex forceSegmentsForRenderingMtx_;

	///  Accumulated noisy odometry.
	mrpt::poses::CPose2D odometry_;
	/// Temporary variable to detect pose change.
	std::optional<mrpt::poses::CPose3D> odometry_pre_simul_pose_;

   public:
	// data logger header entries
	static constexpr std::string_view DL_TIMESTAMP = "Timestamp";
	static constexpr std::size_t LOGGER_IDX_POSE = 0;
	static constexpr std::size_t LOGGER_IDX_WHEELS = 1;

	static constexpr std::string_view PL_Q_X = "q0x";
	static constexpr std::string_view PL_Q_Y = "q1y";
	static constexpr std::string_view PL_Q_Z = "q2z";
	static constexpr std::string_view PL_Q_YAW = "q3yaw";
	static constexpr std::string_view PL_Q_PITCH = "q4pitch";
	static constexpr std::string_view PL_Q_ROLL = "q5roll";
	static constexpr std::string_view PL_DQ_X = "dqx";
	static constexpr std::string_view PL_DQ_Y = "dqy";
	static constexpr std::string_view PL_DQ_Z = "dqz";

	static constexpr std::string_view PL_ODO_X = "odo_x";
	static constexpr std::string_view PL_ODO_Y = "odo_y";
	static constexpr std::string_view PL_ODO_YAW = "odo_yaw";

	static constexpr std::string_view WL_TORQUE = "torque";
	static constexpr std::string_view WL_FORCE_Z = "force_z";
	static constexpr std::string_view WL_VEL_X = "velocity_x";
	static constexpr std::string_view WL_VEL_Y = "velocity_y";
	static constexpr std::string_view WL_FRIC_X = "friction_x";
	static constexpr std::string_view WL_FRIC_Y = "friction_y";

	bool isLogging() const;
};	// end VehicleBase

// Class factory:
typedef ClassFactory<VehicleBase, World*> TClassFactory_vehicleDynamics;
extern TClassFactory_vehicleDynamics classFactory_vehicleDynamics;

#define DECLARES_REGISTER_VEHICLE_DYNAMICS(CLASS_NAME) \
	DECLARES_REGISTER_CLASS1(CLASS_NAME, VehicleBase, World*)

#define REGISTER_VEHICLE_DYNAMICS(TEXTUAL_NAME, CLASS_NAME) \
	REGISTER_CLASS1(                                        \
		TClassFactory_vehicleDynamics, classFactory_vehicleDynamics, TEXTUAL_NAME, CLASS_NAME)
}  // namespace mvsim
