set(MODULE_NAME "simulator")
project(mvsim-${MODULE_NAME} LANGUAGES CXX VERSION ${MVSIM_VERSION})

# Declare the target (a library)
add_library(${PROJECT_NAME}
	# global files:
	src/Block.cpp
	src/CollisionShapeCache.cpp
	src/CsvLogger.cpp
	src/JointXMLnode.h
	src/Joystick.cpp
	src/ModelsCache.cpp
	src/ModelsCache.h
	src/parse_utils.cpp
	src/parse_utils.h
	src/PID_Controller.cpp
	src/RemoteResourcesManager.cpp
	src/Shape2p5.cpp
	src/Simulable.cpp
	src/VehicleBase.cpp
	src/World.cpp
	src/World_gui.cpp
	src/World_load_xml.cpp
	src/World_services.cpp
	src/World_simul.cpp
	src/World_walls.cpp
	src/XMLClassesRegistry.cpp
	src/XMLClassesRegistry.h
	src/xml_utils.cpp
	src/xml_utils.h
	src/VisualObject.cpp
	src/Wheel.cpp
	include/mvsim/basic_types.h
	include/mvsim/Block.h
	include/mvsim/ClassFactory.h
	include/mvsim/CollisionShapeCache.h
	include/mvsim/ControllerBase.h
	include/mvsim/CsvLogger.h
	include/mvsim/Joystick.h
	include/mvsim/mvsim.h
	include/mvsim/mvsim_version.h
	include/mvsim/PID_Controller.h
	include/mvsim/RemoteResourcesManager.h
	include/mvsim/Shape2p5.h
	include/mvsim/Simulable.h
	include/mvsim/TParameterDefinitions.h
	include/mvsim/VehicleBase.h
	include/mvsim/VisualObject.h
	include/mvsim/Wheel.h
	include/mvsim/World.h

	# FrictionModels:
	src/FrictionModels/DefaultFriction.cpp
	src/FrictionModels/FrictionBase.cpp
	src/FrictionModels/WardIagnemmaFriction.cpp
	include/mvsim/FrictionModels/DefaultFriction.h
	include/mvsim/FrictionModels/FrictionBase.h
	include/mvsim/FrictionModels/WardIagnemmaFriction.h

	# Sensors:
	src/Sensors/CameraSensor.cpp
	src/Sensors/DepthCameraSensor.cpp
	src/Sensors/IMU.cpp
	src/Sensors/GNSS.cpp
	src/Sensors/LaserScanner.cpp
	src/Sensors/Lidar3D.cpp
	src/Sensors/SensorBase.cpp
	include/mvsim/Sensors/CameraSensor.h
	include/mvsim/Sensors/DepthCameraSensor.h
	include/mvsim/Sensors/IMU.h
	include/mvsim/Sensors/GNSS.h
	include/mvsim/Sensors/LaserScanner.h
	include/mvsim/Sensors/Lidar3D.h
	include/mvsim/Sensors/SensorBase.h

	# VehicleDynamics:
	src/VehicleDynamics/VehicleAckermann_ControllerFrontSteerPID.cpp
	src/VehicleDynamics/VehicleAckermann_ControllerRaw.cpp
	src/VehicleDynamics/VehicleAckermann_ControllerTwistFrontSteerPID.cpp
	src/VehicleDynamics/VehicleAckermann.cpp
	src/VehicleDynamics/VehicleAckermann_Drivetrain_ControllerFrontSteerPID.cpp
	src/VehicleDynamics/VehicleAckermann_Drivetrain_ControllerRaw.cpp
	src/VehicleDynamics/Vehicleackermann_Drivetrain_ControllerTwistFrontSteerPID.cpp
	src/VehicleDynamics/VehicleAckermann_Drivetrain.cpp
	src/VehicleDynamics/VehicleDifferential_ControllerRaw.cpp
	src/VehicleDynamics/VehicleDifferential_ControllerTwistIdeal.cpp
	src/VehicleDynamics/VehicleDifferential_ControllerTwistPID.cpp
	src/VehicleDynamics/VehicleDifferential.cpp
	include/mvsim/VehicleDynamics/VehicleAckermann_Drivetrain.h
	include/mvsim/VehicleDynamics/VehicleAckermann.h
	include/mvsim/VehicleDynamics/VehicleDifferential.h

	# WorldElements:
	src/WorldElements/ElevationMap.cpp
	src/WorldElements/GroundGrid.cpp
	src/WorldElements/HorizontalPlane.cpp
	src/WorldElements/OccupancyGridMap.cpp
	src/WorldElements/PointCloud.cpp
	src/WorldElements/SkyBox.cpp
	src/WorldElements/VerticalPlane.cpp
	src/WorldElements/WorldElementBase.cpp
	include/mvsim/WorldElements/ElevationMap.h
	include/mvsim/WorldElements/GroundGrid.h
	include/mvsim/WorldElements/HorizontalPlane.h
	include/mvsim/WorldElements/OccupancyGridMap.h
	include/mvsim/WorldElements/PointCloud.h
	include/mvsim/WorldElements/SkyBox.h
	include/mvsim/WorldElements/VerticalPlane.h
	include/mvsim/WorldElements/WorldElementBase.h
)

add_library(mvsim::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

mvsim_set_target_build_options(${PROJECT_NAME})

source_group(Main FILES ${ROOT_SRCS})
source_group(Sensors FILES ${Sensors_SRCS})
source_group(Friction FILES ${Friction_SRCS})
source_group(VehicleDynamics FILES ${VehicleDynamics_SRCS})
source_group(WorldElements FILES ${WorldElements_SRCS})

if (TARGET mvsim::comms)
    set(DEP_MVSIM_COMMS_PKG mvsim-comms)
    set(DEP_MVSIM_COMMS_TRG mvsim::comms)
endif()
if (TARGET mvsim::msgs)
    set(DEP_MVSIM_MSGS_TRG mvsim::msgs)
endif()

set(PACKAGE_DEPENDENCIES
  mrpt-maps
  mrpt-gui
  mrpt-topography
  ${DEP_MVSIM_COMMS_PKG}
  box2d
  )
mvsim_common_target_settings(${MODULE_NAME})


target_link_libraries(${PROJECT_NAME}
PRIVATE
	${DEP_MVSIM_MSGS_TRG}
PUBLIC
	mrpt::obs
	mrpt::opengl
	mrpt::maps
	mrpt::gui
	mrpt::topography
	${DEP_MVSIM_COMMS_TRG}
	${BOX2D_LIBRARIES}
)


target_include_directories(${PROJECT_NAME}
	SYSTEM PUBLIC
		$<BUILD_INTERFACE:${mvsim_SOURCE_DIR}/externals/rapidxml>
)
