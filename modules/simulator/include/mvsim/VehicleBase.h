/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
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

#include <map>
#include <mutex>
#include <string>

#include "CsvLogger.h"

namespace mvsim
{
/** Virtual base class for each vehicle "actor" in the simulation.
 * Derived classes implements different dynamical models (Differential,
 * Ackermann,...)
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
		const rapidxml::xml_node<char>* xml_node);

	void poses_mutex_lock() override { m_gui_mtx.lock(); }
	void poses_mutex_unlock() override { m_gui_mtx.unlock(); }

	// ------- Interface with "World" ------
	virtual void simul_pre_timestep(const TSimulContext& context) override;
	virtual void simul_post_timestep(const TSimulContext& context) override;
	virtual void apply_force(
		const mrpt::math::TVector2D& force,
		const mrpt::math::TPoint2D& applyPoint =
			mrpt::math::TPoint2D(0, 0)) override;

	/** Create bodies, fixtures, etc. for the dynamical simulation. May be
	 * overrided by derived classes */
	virtual void create_multibody_system(b2World& world);

	/** Get (an approximation of) the max radius of the vehicle, from its point
	 * of reference (in meters) */
	virtual float getMaxVehicleRadius() const { return m_max_radius; }
	/** Get the overall vehicle mass, excluding wheels. */
	virtual double getChassisMass() const { return m_chassis_mass; }
	b2Body* getBox2DChassisBody() { return m_b2d_body; }
	mrpt::math::TPoint2D getChassisCenterOfMass() const
	{
		return m_chassis_com;
	}  //!< In local coordinates (this excludes the mass of wheels)

	size_t getNumWheels() const { return m_wheels_info.size(); }
	const Wheel& getWheelInfo(const size_t idx) const
	{
		return m_wheels_info[idx];
	}
	Wheel& getWheelInfo(const size_t idx) { return m_wheels_info[idx]; }

	/** Current velocity of each wheel's center point (in local coords). Call
	 * with veh_vel_local=getVelocityLocal() for ground-truth.  */
	void getWheelsVelocityLocal(
		std::vector<mrpt::math::TPoint2D>& vels,
		const mrpt::math::TTwist2D& veh_vel_local) const;

	/** Gets the current estimation of odometry-based velocity as reconstructed
	 * solely from wheels spinning velocities and geometry.
	 * This is the input of any realistic low-level controller onboard.
	 * \sa getVelocityLocal() */
	virtual mrpt::math::TTwist2D getVelocityLocalOdoEstimate() const = 0;

	const TListSensors& getSensors() const { return m_sensors; }
	TListSensors& getSensors() { return m_sensors; }
	std::shared_ptr<CSVLogger> getLoggerPtr(std::string logger_name)
	{
		return m_loggers[logger_name];
	}

	/** Get the 2D shape of the vehicle chassis, as set from the config file
	 * (only used for collision detection) */
	const mrpt::math::TPolygon2D& getChassisShape() const
	{
		return m_chassis_poly;
	}

	/** Set the vehicle index in the World */
	void setVehicleIndex(size_t idx) { m_vehicle_index = idx; }
	/** Get the vehicle index in the World */
	size_t getVehicleIndex() const { return m_vehicle_index; }
	void setRecording(bool record)
	{
		for (auto& logger : m_loggers) logger.second->setRecording(record);
	}
	void clearLogs()
	{
		for (auto& logger : m_loggers) logger.second->clear();
	}
	void newLogSession()
	{
		for (auto& logger : m_loggers) logger.second->newSession();
	}

	virtual ControllerBaseInterface* getControllerInterface() = 0;

	void registerOnServer(mvsim::Client& c) override;

	b2Fixture* get_fixture_chassis() { return m_fixture_chassis; }
	std::vector<b2Fixture*>& get_fixture_wheels() { return m_fixture_wheels; }
	const b2Fixture* get_fixture_chassis() const { return m_fixture_chassis; }
	const std::vector<b2Fixture*>& get_fixture_wheels() const
	{
		return m_fixture_wheels;
	}

	void freeOpenGLResources() override
	{
		for (auto& sensor : m_sensors) sensor->freeOpenGLResources();
	}

   protected:
	std::map<std::string, std::shared_ptr<CSVLogger>> m_loggers;
	std::string m_log_path;

	virtual void initLoggers();
	virtual void writeLogStrings();
	virtual void internalGuiUpdate(
		mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical,
		bool childrenOnly) override;
	virtual mrpt::poses::CPose3D internalGuiGetVisualPose() override;

   protected:
	// Protected ctor for class factory
	VehicleBase(World* parent, size_t nWheels);

	/** Parse node <dynamics>: The derived-class part of load_params_from_xml(),
	 * also called in factory(). Includes parsing the <controller></controller>
	 * block. */
	virtual void dynamics_load_params_from_xml(
		const rapidxml::xml_node<char>* xml_node) = 0;

	virtual void invoke_motor_controllers(
		const TSimulContext& context,
		std::vector<double>& out_force_per_wheel) = 0;

	/** user-supplied index number: must be set/get'ed with setVehicleIndex()
	 * getVehicleIndex() (default=0) */
	size_t m_vehicle_index = 0;

	/** Instance of friction model for the vehicle-to-ground interaction. */
	FrictionBasePtr m_friction;

	TListSensors m_sensors;	 //!< Sensors aboard

	/** Updated in simul_pre_timestep() */
	std::vector<double> m_torque_per_wheel;

	// Chassis info:
	double m_chassis_mass = 15.0;
	mrpt::math::TPolygon2D m_chassis_poly;

	/** Automatically computed from m_chassis_poly upon each change via
	 * updateMaxRadiusFromPoly() */
	double m_max_radius = 0.1;

	double m_chassis_z_min = 0.05, m_chassis_z_max = 0.6;

	mrpt::img::TColor m_chassis_color{0xff, 0x00, 0x00};

	/** center of mass. in local coordinates (this excludes the mass of wheels)
	 */
	mrpt::math::TPoint2D m_chassis_com{0, 0};

	void updateMaxRadiusFromPoly();

	/** Wheels info.  The fixed size of this vector is set upon construction.
	 *  Derived classes must define the order of the wheels, e.g. [0]=rear left,
	 * etc.
	 */
	std::deque<Wheel> m_wheels_info;

	// Box2D elements:
	b2Fixture* m_fixture_chassis;  //!< Created at

	/** [0]:rear-left, etc. (depending on derived class). Size set at
	 * constructor. */
	std::vector<b2Fixture*> m_fixture_wheels;

   private:
	// Called from internalGuiUpdate_common()
	void internal_internalGuiUpdate_sensors(
		mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical);
	// Called from internalGuiUpdate_common()
	void internal_internalGuiUpdate_forces(mrpt::opengl::COpenGLScene& scene);

	mrpt::opengl::CSetOfObjects::Ptr m_gl_chassis;
	std::vector<mrpt::opengl::CSetOfObjects::Ptr> m_gl_wheels;
	mrpt::opengl::CSetOfLines::Ptr m_gl_forces;
	std::mutex m_force_segments_for_rendering_cs;
	std::vector<mrpt::math::TSegment3D> m_force_segments_for_rendering;

	std::recursive_mutex m_gui_mtx;

   public:	// data logger header entries
	static constexpr char DL_TIMESTAMP[] = "timestamp";
	static constexpr char LOGGER_POSE[] = "logger_pose";
	static constexpr char LOGGER_WHEEL[] = "logger_wheel";

	static constexpr char PL_Q_X[] = "Qx";
	static constexpr char PL_Q_Y[] = "Qy";
	static constexpr char PL_Q_Z[] = "Qz";
	static constexpr char PL_Q_YAW[] = "Qyaw";
	static constexpr char PL_Q_PITCH[] = "Qpitch";
	static constexpr char PL_Q_ROLL[] = "Qroll";
	static constexpr char PL_DQ_X[] = "dQx";
	static constexpr char PL_DQ_Y[] = "dQy";
	static constexpr char PL_DQ_Z[] = "dQz";

	static constexpr char WL_TORQUE[] = "torque";
	static constexpr char WL_WEIGHT[] = "weight";
	static constexpr char WL_VEL_X[] = "velocity_x";
	static constexpr char WL_VEL_Y[] = "velocity_y";
	static constexpr char WL_FRIC_X[] = "friction_x";
	static constexpr char WL_FRIC_Y[] = "friction_y";
};	// end VehicleBase

// Class factory:
typedef ClassFactory<VehicleBase, World*> TClassFactory_vehicleDynamics;
extern TClassFactory_vehicleDynamics classFactory_vehicleDynamics;

#define DECLARES_REGISTER_VEHICLE_DYNAMICS(CLASS_NAME) \
	DECLARES_REGISTER_CLASS1(CLASS_NAME, VehicleBase, World*)

#define REGISTER_VEHICLE_DYNAMICS(TEXTUAL_NAME, CLASS_NAME)          \
	REGISTER_CLASS1(                                                 \
		TClassFactory_vehicleDynamics, classFactory_vehicleDynamics, \
		TEXTUAL_NAME, CLASS_NAME)
}  // namespace mvsim
