/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/format.h>
#include <mrpt/core/get_env.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/core/round.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/version.h>
#include <mvsim/World.h>

#include <cmath>  // cos(), sin()
#include <rapidxml.hpp>

#include "xml_utils.h"
#if MRPT_VERSION >= 0x204
#include <mrpt/system/thread_name.h>
#endif

using namespace mvsim;
using namespace std;

void World::TGUI_Options::parse_from(
	const rapidxml::xml_node<char>& node, mrpt::system::COutputLogger& logger)
{
	parse_xmlnode_children_as_param(
		node, params, {}, "[World::TGUI_Options]", &logger);
}

void World::LightOptions::parse_from(
	const rapidxml::xml_node<char>& node, mrpt::system::COutputLogger& logger)
{
	parse_xmlnode_children_as_param(
		node, params, {}, "[World::LightOptions]", &logger);
}

//!< Return true if the GUI window is open, after a previous call to
//! update_GUI()
bool World::is_GUI_open() const { return !!gui_.gui_win; }
//!< Forces closing the GUI window, if any.
void World::close_GUI() { gui_.gui_win.reset(); }

// Add top menu subwindow:
void World::GUI::prepare_control_window()
{
#if MRPT_VERSION >= 0x211
	nanogui::Window* w = gui_win->createManagedSubWindow("Control");
#else
	nanogui::Window* w = new nanogui::Window(gui_win.get(), "Control");
#endif

	// Place control UI at the top-left corner:
	gui_win->getSubWindowsUI()->setPosition({1, 1});

	w->setPosition({1, 80});
	w->setLayout(new nanogui::BoxLayout(
		nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5));

	w->add<nanogui::Button>("Quit", ENTYPO_ICON_ARROW_BOLD_LEFT)
		->setCallback([this]() {
			parent_.simulator_must_close(true);
			gui_win->setVisible(false);
			nanogui::leave();
		});

	std::vector<std::string> lstVehicles;
	lstVehicles.reserve(parent_.vehicles_.size() + 1);

	lstVehicles.push_back("[none]");  // None
	for (const auto& v : parent_.vehicles_) lstVehicles.push_back(v.first);

	w->add<nanogui::Label>("Camera follows:");
	auto cbFollowVeh = w->add<nanogui::ComboBox>(lstVehicles);
	cbFollowVeh->setSelectedIndex(0);
	cbFollowVeh->setCallback([this, lstVehicles](int idx) {
		if (idx == 0)
			parent_.guiOptions_.follow_vehicle.clear();
		else if (idx <= static_cast<int>(parent_.vehicles_.size()))
			parent_.guiOptions_.follow_vehicle = lstVehicles[idx];
	});

	w->add<nanogui::CheckBox>("Orthogonal view", [&](bool b) {
		 gui_win->camera().setCameraProjective(!b);
	 })->setChecked(parent_.guiOptions_.ortho);

#if MRPT_VERSION >= 0x270
	w->add<nanogui::CheckBox>("Enable shadows", [&](bool b) {
		 auto vv = parent_.worldVisual_->getViewport();
		 auto vp = parent_.worldPhysical_.getViewport();
		 vv->enableShadowCasting(b);
		 vp->enableShadowCasting(b);
		 parent_.lightOptions_.enable_shadows = b;
	 })->setChecked(parent_.lightOptions_.enable_shadows);
#endif

	w->add<nanogui::Label>("Light azimuth:");
	{
		auto sl = w->add<nanogui::Slider>();
		sl->setRange({-M_PI, M_PI});
		sl->setValue(parent_.lightOptions_.light_azimuth);
		sl->setCallback([this](float v) {
			parent_.lightOptions_.light_azimuth = v;
			parent_.setLightDirectionFromAzimuthElevation(
				parent_.lightOptions_.light_azimuth,
				parent_.lightOptions_.light_elevation);
		});
	}
	w->add<nanogui::Label>("Light elevation:");
	{
		auto sl = w->add<nanogui::Slider>();
		sl->setRange({0, M_PI * 0.5});
		sl->setValue(parent_.lightOptions_.light_elevation);
		sl->setCallback([this](float v) {
			parent_.lightOptions_.light_elevation = v;
			parent_.setLightDirectionFromAzimuthElevation(
				parent_.lightOptions_.light_azimuth,
				parent_.lightOptions_.light_elevation);
		});
	}

	w->add<nanogui::CheckBox>("View forces", [&](bool b) {
		 parent_.guiOptions_.show_forces = b;
	 })->setChecked(parent_.guiOptions_.show_forces);

	w->add<nanogui::CheckBox>("View sensor pointclouds", [&](bool b) {
		 std::lock_guard<std::mutex> lck(gui_win->background_scene_mtx);

		 auto glVizSensors =
			 std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(
				 gui_win->background_scene->getByName("group_sensors_viz"));
		 ASSERT_(glVizSensors);

		 glVizSensors->setVisibility(b);
	 })->setChecked(parent_.guiOptions_.show_sensor_points);

	w->add<nanogui::CheckBox>("View sensor poses", [&](bool b) {
		 const auto& objs = SensorBase::GetAllSensorsOriginViz();
		 for (const auto& o : *objs) o->setVisibility(b);
	 })->setChecked(false);

	w->add<nanogui::CheckBox>("View sensor FOVs", [&](bool b) {
		 const auto& objs = SensorBase::GetAllSensorsFOVViz();
		 for (const auto& o : *objs) o->setVisibility(b);
	 })->setChecked(false);

	w->add<nanogui::CheckBox>("View collision shapes", [&](bool b) {
		 auto lck = mrpt::lockHelper(parent_.simulableObjectsMtx_);
		 for (auto& s : parent_.simulableObjects_)
		 {
			 auto* vis = dynamic_cast<VisualObject*>(s.second.get());
			 if (!vis) continue;
			 vis->showCollisionShape(b);
		 }
	 })->setChecked(false);
}

// Add Status window
void World::GUI::prepare_status_window()
{
#if MRPT_VERSION >= 0x211
	nanogui::Window* w = gui_win->createManagedSubWindow("Status");
#else
	nanogui::Window* w = new nanogui::Window(gui_win.get(), "Status");
#endif

	w->setPosition({5, 455});
	w->setLayout(new nanogui::BoxLayout(
		nanogui::Orientation::Vertical, nanogui::Alignment::Fill));
	w->setFixedWidth(320);

#if MRPT_VERSION < 0x211
	w->buttonPanel()
		->add<nanogui::Button>("", ENTYPO_ICON_CROSS)
		->setCallback([w]() { w->setVisible(false); });
#endif

	lbCpuUsage = w->add<nanogui::Label>(" ");
	lbStatuses.resize(12);
	for (size_t i = 0; i < lbStatuses.size(); i++)
		lbStatuses[i] = w->add<nanogui::Label>(" ");
}

// Add editor window
void World::GUI::prepare_editor_window()
{
#if MRPT_VERSION >= 0x211
#if MRPT_VERSION >= 0x231
	const auto subwinIdx = gui_win->getSubwindowCount();
#endif
	nanogui::Window* w = gui_win->createManagedSubWindow("Editor");
#else
	nanogui::Window* w = new nanogui::Window(gui_win.get(), "Editor");
#endif

	constexpr int pnWidth = 300, pnHeight = 200;
	constexpr int COORDS_LABEL_WIDTH = 60;
	constexpr int slidersWidth = pnWidth - 80 - COORDS_LABEL_WIDTH;

	w->setPosition({1, 230});
	w->setLayout(new nanogui::BoxLayout(
		nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 3, 3));
	w->setFixedWidth(pnWidth);

#if MRPT_VERSION < 0x211
	w->buttonPanel()
		->add<nanogui::Button>("", ENTYPO_ICON_CROSS)
		->setCallback([w]() { w->setVisible(false); });
#endif

	w->add<nanogui::Label>("Selected object", "sans-bold");

	// Auxiliary lambda placeholder for when the user clicks on an object
	// being able to load its current pose in the GUI controls yet to be
	// constructed later on:
	static std::function<void(const mrpt::math::TPose3D)> onEntitySelected;
	static std::function<void(const mrpt::math::TPose3D)> onEntityMoved;

	auto lckListObjs = mrpt::lockHelper(parent_.getListOfSimulableObjectsMtx());
	if (!parent_.getListOfSimulableObjects().empty())
	{
		auto tab = w->add<nanogui::TabWidget>();

		constexpr size_t NUM_TABS = 5;

		std::array<nanogui::Widget*, NUM_TABS> tabs = {
			tab->createTab("Vehicles"), tab->createTab("Sensors"),
			tab->createTab("Blocks"), tab->createTab("Elements"),
			tab->createTab("Misc.")};

		tab->setActiveTab(0);

		for (auto t : tabs)
			t->setLayout(new nanogui::BoxLayout(
				nanogui::Orientation::Vertical, nanogui::Alignment::Minimum, 3,
				3));

		std::array<nanogui::VScrollPanel*, NUM_TABS> vscrolls;
		for (size_t i = 0; i < NUM_TABS; i++)
			vscrolls[i] = tabs[i]->add<nanogui::VScrollPanel>();

		for (auto vs : vscrolls) vs->setFixedSize({pnWidth, pnHeight});

		// vscroll should only have *ONE* child. this is what `wrapper`
		// is for
		std::array<nanogui::Widget*, NUM_TABS> wrappers;
		for (size_t i = 0; i < NUM_TABS; i++)
		{
			wrappers[i] = vscrolls[i]->add<nanogui::Widget>();
			wrappers[i]->setFixedSize({pnWidth, pnHeight});
			wrappers[i]->setLayout(new nanogui::GridLayout(
				nanogui::Orientation::Horizontal, 1 /*columns */,
				nanogui::Alignment::Minimum, 3, 3));
		}

		// Extend the list of world objects with the robot sensors:
		SimulableList listAllObjs = parent_.getListOfSimulableObjects();
		// Yes: we iterate over parent.getList...(), *not* the local copy, since
		// it will be modified within the loop.
		for (const auto& o : parent_.getListOfSimulableObjects())
		{
			if (auto v = dynamic_cast<VehicleBase*>(o.second.get()); v)
			{
				auto& sensors = v->getSensors();
				for (auto& sensor : sensors)
				{
					const auto sensorFullName =
						sensor->vehicle().getName() + "."s + sensor->getName();
					listAllObjs.insert({sensorFullName, sensor});
				}
			}
		}

		// Now, fill the editor list with all the existing objects:
		for (const auto& o : listAllObjs)
		{
			InfoPerObject ipo;

			const auto& name = o.first;
			int wrapperIdx = -1;  // default. The tag "page" to show this at.
			if (auto v = dynamic_cast<VehicleBase*>(o.second.get()); v)
			{
				wrapperIdx = 0;
				ipo.visual = dynamic_cast<VisualObject*>(v);
			}
			if (auto v = dynamic_cast<Block*>(o.second.get()); v)
			{
				wrapperIdx = 2;
				ipo.visual = dynamic_cast<VisualObject*>(v);
			}
			if (auto v = dynamic_cast<SensorBase*>(o.second.get()); v)
			{
				wrapperIdx = 1;
				ipo.visual = dynamic_cast<VisualObject*>(v);
			}
			// bool isWorldElement = false;
			if (auto v = dynamic_cast<WorldElementBase*>(o.second.get()); v)
			{
				// isWorldElement = true;
				wrapperIdx = 3;
				ipo.visual = dynamic_cast<VisualObject*>(v);
			}

			if (wrapperIdx < 0) continue;  // unknown / non-editable item.

			auto wrapper = wrappers[wrapperIdx];

			std::string label = name;
			if (label.empty()) label = "(unnamed)";

			auto cb = wrapper->add<nanogui::CheckBox>(label);
			ipo.cb = cb;
			ipo.simulable = o.second;
			gui_cbObjects.emplace_back(ipo);

			cb->setChecked(false);
			cb->setCallback([cb, ipo, this](bool check) {
				// deselect former one:
				if (gui_selectedObject.visual)
					gui_selectedObject.visual->showCollisionShape(false);
				if (gui_selectedObject.cb)
					gui_selectedObject.cb->setChecked(false);
				gui_selectedObject = InfoPerObject();

				cb->setChecked(check);

				// If checked, show bounding box:
				if (ipo.visual && check)
				{
					gui_selectedObject = ipo;
					ipo.visual->showCollisionShape(true);
				}

				const bool btnsEnabled = !!gui_selectedObject.simulable;
				for (auto b : btns_selectedOps) b->setEnabled(btnsEnabled);

				// Set current coordinates in controls:
				if (ipo.simulable && onEntitySelected)
					onEntitySelected(ipo.simulable->getRelativePose());
			});
		}

		// "misc." tab
		// --------------
		wrappers[4]
			->add<nanogui::Button>("Save 3D scene...", ENTYPO_ICON_EXPORT)
			->setCallback([this]() {
				try
				{
					const std::string outFile = nanogui::file_dialog(
						{{"3Dscene", "MRPT 3D scene file (*.3Dsceme)"}},
						true /*save*/);
					if (outFile.empty()) return;

					auto lck = mrpt::lockHelper(parent_.physical_objects_mtx());
					parent_.worldPhysical_.saveToFile(outFile);

					std::cout << "[mvsim gui] Saved world scene to: " << outFile
							  << std::endl;
				}
				catch (const std::exception& e)
				{
					std::cerr
						<< "[mvsim gui] Exception while saving 3D scene:\n"
						<< e.what() << std::endl;
				}
			});
	}

	w->add<nanogui::Label>(" ");

	// Replace with mouse:
	btnReplaceObject = w->add<nanogui::Button>("Click to replace...");
	btnReplaceObject->setFlags(nanogui::Button::Flags::ToggleButton);
	btns_selectedOps.push_back(btnReplaceObject);

	// Reorient (yaw/pitch/roll):
	constexpr float REPOSITION_SLIDER_RANGE = 1.0;	// Meters

	nanogui::Slider* slidersCoordScale = nullptr;
	nanogui::Label* slidersCoordScaleValue = nullptr;
	{
		auto pn = w->add<nanogui::Widget>();
		pn->setLayout(new nanogui::BoxLayout(
			nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 2, 2));
		pn->add<nanogui::Label>("Change scale:");

		auto slCoord = pn->add<nanogui::Slider>();
		slidersCoordScale = slCoord;

		slCoord->setRange({-4.0, 1.0});

		slCoord->setCallback([this]([[maybe_unused]] float v) {
			// Re-generate the other 6 sliders with this new scale:
			if (!gui_selectedObject.simulable || !onEntitySelected) return;
			onEntitySelected(gui_selectedObject.simulable->getRelativePose());
		});
		slCoord->setFixedWidth(slidersWidth - 30);
		btns_selectedOps.push_back(slCoord);

		slidersCoordScaleValue = pn->add<nanogui::Label>(" ");
		slidersCoordScaleValue->setFixedWidth(70);
	}

	std::array<nanogui::Slider*, 6> slidersCoords;
	std::array<nanogui::Label*, 6> slidersCoordsValues;
	const std::array<const char*, 6> coordsNames = {
		"    Move 'x':",  //
		"    Move 'y':",  //
		"    Move 'z':",  //
		"Rotate   yaw:",  //
		"Rotate pitch:",  //
		"Rotate  roll:"};

	for (int axis = 0; axis < 6; axis++)
	{
		auto pn = w->add<nanogui::Widget>();
		pn->setLayout(new nanogui::BoxLayout(
			nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 2, 2));
		pn->add<nanogui::Label>(coordsNames[axis]);

		auto slCoord = pn->add<nanogui::Slider>();
		slidersCoords[axis] = slCoord;

		// Dummy. Correct ones set in onEntitySelected()
		slCoord->setRange({-1.0, 1.0});

		slCoord->setCallback([this, axis](float v) {
			if (!gui_selectedObject.simulable) return;
			auto p = gui_selectedObject.simulable->getRelativePose();
			p[axis] = v;
			gui_selectedObject.simulable->setRelativePose(p);
			onEntityMoved(p);
		});
		slCoord->setFixedWidth(slidersWidth);
		btns_selectedOps.push_back(slCoord);

		slidersCoordsValues[axis] = pn->add<nanogui::Label>("(...)");
		slidersCoordsValues[axis]->setFixedWidth(COORDS_LABEL_WIDTH);
	}

	// Now, we can define the lambda for filling in the current object pose in
	// the GUI controls:
	onEntitySelected = [slidersCoords, slidersCoordScale,
						slidersCoordScaleValue](const mrpt::math::TPose3D p) {
		ASSERT_(slidersCoordScale);
		const double scale =
			std::pow(10.0, mrpt::round(slidersCoordScale->value()));

		slidersCoordScaleValue->setCaption(mrpt::format("%.01e", scale));

		// Positions:
		for (int i = 0; i < 3; i++)
		{
			slidersCoords[i]->setRange(
				{p[i] - scale * REPOSITION_SLIDER_RANGE,
				 p[i] + scale * REPOSITION_SLIDER_RANGE});
			slidersCoords[i]->setValue(p[i]);
		}
		// Angles:
		for (int i = 0; i < 3; i++)
		{
			slidersCoords[i + 3]->setRange(
				{p[i + 3] - scale * M_PI, p[i + 3] + scale * M_PI});
			slidersCoords[i + 3]->setValue(p[i + 3]);
		}

		onEntityMoved(p);
	};

	onEntityMoved = [slidersCoordsValues](const mrpt::math::TPose3D p) {
		// Positions:
		for (int i = 0; i < 3; i++)
			slidersCoordsValues[i]->setCaption(mrpt::format("%.04f", p[i]));
		// Angles:
		for (int i = 0; i < 3; i++)
			slidersCoordsValues[i + 3]->setCaption(
				mrpt::format("%.02f deg", mrpt::RAD2DEG(p[i + 3])));
	};

	// Replace with coordinates:
	auto btnPlaceCoords = w->add<nanogui::Button>("Replace by coordinates...");
	btns_selectedOps.push_back(btnPlaceCoords);
	btnPlaceCoords->setCallback([this]() {
		//
		if (!gui_selectedObject.simulable) return;

		auto* formPose = new nanogui::Window(gui_win.get(), "Enter new pose");
		formPose->setLayout(new nanogui::GridLayout(
			nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill, 5));

		nanogui::TextBox* lbs[6];

		formPose->add<nanogui::Label>("x:");
		lbs[0] = formPose->add<nanogui::TextBox>();
		formPose->add<nanogui::Label>("y:");
		lbs[1] = formPose->add<nanogui::TextBox>();
		formPose->add<nanogui::Label>("z:");
		lbs[2] = formPose->add<nanogui::TextBox>();
		formPose->add<nanogui::Label>("Yaw:");
		lbs[3] = formPose->add<nanogui::TextBox>();
		formPose->add<nanogui::Label>("Pitch:");
		lbs[4] = formPose->add<nanogui::TextBox>();
		formPose->add<nanogui::Label>("Roll:");
		lbs[5] = formPose->add<nanogui::TextBox>();

		for (int i = 0; i < 6; i++)
		{
			lbs[i]->setEditable(true);
			lbs[i]->setFixedSize({100, 20});
			lbs[i]->setValue("0.0");
			lbs[i]->setUnits(i >= 3 ? "[deg]" : "[m]");
			lbs[i]->setDefaultValue("0.0");
			lbs[i]->setFontSize(16);
			lbs[i]->setFormat("[-]?[0-9]*\\.?[0-9]+");
		}

		const auto pos = gui_selectedObject.simulable->getRelativePose();
		for (int i = 0; i < 3; i++) lbs[i]->setValue(std::to_string(pos[i]));

		for (int i = 3; i < 6; i++)
			lbs[i]->setValue(std::to_string(mrpt::RAD2DEG(pos[i])));

		formPose->add<nanogui::Label>("");
		formPose->add<nanogui::Label>("");

		formPose->add<nanogui::Button>("Cancel")->setCallback(
			[formPose]() { formPose->dispose(); });

		formPose->add<nanogui::Button>("Accept")->setCallback(
			[formPose, this, lbs]() {
				const mrpt::math::TPose3D newPose = {
					// X:
					std::stod(lbs[0]->value()),
					// Y:
					std::stod(lbs[1]->value()),
					// Z:
					std::stod(lbs[2]->value()),
					// Yaw
					mrpt::DEG2RAD(std::stod(lbs[3]->value())),
					// Pitch
					mrpt::DEG2RAD(std::stod(lbs[4]->value())),
					// Roll:
					mrpt::DEG2RAD(std::stod(lbs[5]->value()))};

				gui_selectedObject.simulable->setRelativePose(newPose);
				onEntitySelected(newPose);

				formPose->dispose();
			});

		formPose->setModal(true);
		formPose->center();
		formPose->setVisible(true);
	});

	// Disable all edit-controls since no object is selected:
	for (auto b : btns_selectedOps) b->setEnabled(false);

		// Minimize subwindow:
#if MRPT_VERSION >= 0x231
	gui_win->subwindowMinimize(subwinIdx);
#else
	if (auto btnMinimize =
			dynamic_cast<nanogui::Button*>(w->buttonPanel()->children().at(0));
		btnMinimize)
	{
		btnMinimize->callback()();	// "push" button
	}
#endif

}  // end "editor" window

void World::internal_GUI_thread()
{
	try
	{
		MRPT_LOG_DEBUG("[World::internal_GUI_thread] Started.");

		// Start GUI:
		nanogui::init();

		mrpt::gui::CDisplayWindowGUI_Params cp;
		cp.maximized = guiOptions_.start_maximized;

		gui_.gui_win = mrpt::gui::CDisplayWindowGUI::Create(
			"mvsim", guiOptions_.win_w, guiOptions_.win_h, cp);

		// zmin / zmax of opengl viewport:
		worldVisual_->getViewport()->setViewportClipDistances(
			guiOptions_.clip_plane_min, guiOptions_.clip_plane_max);

		// Add a background scene:
		{
			// we use the member scene worldVisual_ as the placeholder for the
			// visual 3D scene:

			// add the placeholders for user-provided objects, both for pure
			// visualization only, and physical objects:
			worldVisual_->insert(glUserObjsViz_);
			worldPhysical_.insert(glUserObjsPhysical_);

			std::lock_guard<std::mutex> lck(gui_.gui_win->background_scene_mtx);
			gui_.gui_win->background_scene = worldVisual_;
		}

		// Only if the world is empty: at least introduce a ground grid:
		if (worldElements_.empty())
		{
			auto we = WorldElementBase::factory(this, nullptr, "ground_grid");
			worldElements_.push_back(we);
		}

		// Windows:
		gui_.prepare_control_window();
		gui_.prepare_status_window();
		gui_.prepare_editor_window();

		// Finish GUI setup:
		gui_.gui_win->performLayout();
		auto& cam = gui_.gui_win->camera();

		cam.setCameraPointing(0.0f, .0f, .0f);
		cam.setCameraProjective(!guiOptions_.ortho);
		cam.setZoomDistance(guiOptions_.camera_distance);
		cam.setAzimuthDegrees(guiOptions_.camera_azimuth_deg);
		cam.setElevationDegrees(guiOptions_.camera_elevation_deg);
		cam.setCameraFOV(guiOptions_.fov_deg);
		cam.setCameraPointing(
			guiOptions_.camera_point_to.x, guiOptions_.camera_point_to.y,
			guiOptions_.camera_point_to.z);

		const auto& lo = lightOptions_;

		setLightDirectionFromAzimuthElevation(
			lo.light_azimuth, lo.light_elevation);

#if MRPT_VERSION >= 0x270
		auto vv = worldVisual_->getViewport();
		auto vp = worldPhysical_.getViewport();

		auto lambdaSetLightParams =
			[&lo](const mrpt::opengl::COpenGLViewport::Ptr& v) {
				// enable shadows and set the shadow map texture size:
				const int sms = lo.shadow_map_size;
				v->enableShadowCasting(lo.enable_shadows, sms, sms);

				// light color:
				const auto colf = mrpt::img::TColorf(lo.light_color);

				auto& vlp = v->lightParameters();

				vlp.color = colf;

#if MRPT_VERSION >= 0x2A0  // New in mrpt>=2.10.0
				vlp.eyeDistance2lightShadowExtension =
					lo.eye_distance_to_shadow_map_extension;

				vlp.minimum_shadow_map_extension_ratio =
					lo.minimum_shadow_map_extension_ratio;
#endif
				// light view frustrum near/far planes:
				v->setLightShadowClipDistances(
					lo.light_clip_plane_min, lo.light_clip_plane_max);

			// Shadow bias should be proportional to clip range:
#if MRPT_VERSION >= 0x281
				vlp.shadow_bias = lo.shadow_bias;
				vlp.shadow_bias_cam2frag = lo.shadow_bias_cam2frag;
				vlp.shadow_bias_normal = lo.shadow_bias_normal;
#endif
			};

		lambdaSetLightParams(vv);
		lambdaSetLightParams(vp);
#endif

		// Main GUI loop
		// ---------------------
		gui_.gui_win->drawAll();
		gui_.gui_win->setVisible(true);

		// Listen for keyboard events:
#if MRPT_VERSION >= 0x232
		gui_.gui_win->addKeyboardCallback(
#else
		gui_.gui_win->setKeyboardCallback(
#endif
			[&](int key, int /*scancode*/, int action, int modifiers) {
				if (action != GLFW_PRESS && action != GLFW_REPEAT) return false;

				auto lck = mrpt::lockHelper(lastKeyEventMtx_);

				lastKeyEvent_.keycode = key;
				lastKeyEvent_.modifierShift = (modifiers & GLFW_MOD_SHIFT) != 0;
				lastKeyEvent_.modifierCtrl =
					(modifiers & GLFW_MOD_CONTROL) != 0;
				lastKeyEvent_.modifierSuper = (modifiers & GLFW_MOD_SUPER) != 0;
				lastKeyEvent_.modifierAlt = (modifiers & GLFW_MOD_ALT) != 0;

				lastKeyEventValid_ = true;

				return false;
			});

		gui_thread_running_ = true;

		// The GUI must be closed from this same thread. Use a shared atomic
		// bool:
		auto lambdaLoopCallback = [](World& me) {
			if (me.simulator_must_close()) nanogui::leave();

			try
			{
				// Update 3D vehicles, sensors, run render-based sensors, etc:
				me.internalGraphicsLoopTasksForSimulation();

				me.internal_process_pending_gui_user_tasks();

				// handle mouse operations:
				me.gui_.handle_mouse_operations();
			}
			catch (const std::exception& e)
			{
				// In case of an exception in the functions above,
				// abort. Otherwise, the error may repeat over and over forever
				// and the main thread will never know about it.
				me.logStr(mrpt::system::LVL_ERROR, e.what());
				me.simulator_must_close(true);
				me.gui_.gui_win->setVisible(false);
				nanogui::leave();
			}
		};

#if MRPT_VERSION >= 0x232
		gui_.gui_win->addLoopCallback(
#else
		gui_.gui_win->setLoopCallback(
#endif
			[=]() { lambdaLoopCallback(*this); });

		// Register observation callback:
		const auto lambdaOnObservation =
			[this](
				const Simulable& veh, const mrpt::obs::CObservation::Ptr& obs) {
				// obs->getDescriptionAsText(std::cout);
				this->enqueue_task_to_run_in_gui_thread([this, obs, &veh]() {
					internal_gui_on_observation(veh, obs);
				});
			};

		this->registerCallbackOnObservation(lambdaOnObservation);

		// ============= Mainloop =============
		const int refresh_ms =
			std::max(1, mrpt::round(1000 / guiOptions_.refresh_fps));

		MRPT_LOG_DEBUG_FMT(
			"[World::internal_GUI_thread] Using GUI FPS=%i (T=%i ms)",
			guiOptions_.refresh_fps, refresh_ms);

#if MRPT_VERSION >= 0x253
		const int idleLoopTasks_ms = 10;

		nanogui::mainloop(idleLoopTasks_ms, refresh_ms);
#else
		nanogui::mainloop(refresh_ms);
#endif

		MRPT_LOG_DEBUG("[World::internal_GUI_thread] Mainloop ended.");

		// to let other threads know that we are closing:
		simulator_must_close(true);

		// Make sure opengl resources are freed from this thread, not from
		// the main one upon destruction of the last ref to shared_ptr's to
		// opengl classes.
		{
			auto lck = mrpt::lockHelper(gui_.gui_win->background_scene_mtx);
			if (gui_.gui_win->background_scene)
				gui_.gui_win->background_scene->freeOpenGLResources();
		}

		auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());

		for (auto& obj : getListOfSimulableObjects())
			obj.second->freeOpenGLResources();

		lckListObjs.unlock();

		VisualObject::FreeOpenGLResources();

		// Now, destroy window:
		gui_.gui_win.reset();

		nanogui::shutdown();
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM(
			"[internal_GUI_init] Exception: " << mrpt::exception_to_str(e));
	}
	gui_thread_running_ = false;
}

void World::GUI::handle_mouse_operations()
{
	MRPT_START
	if (!gui_win) return;

	mrpt::opengl::COpenGLViewport::Ptr vp;
	{
		auto lck = mrpt::lockHelper(gui_win->background_scene_mtx);
		if (!gui_win->background_scene) return;
		vp = gui_win->background_scene->getViewport();
	}
	ASSERT_(vp);

	const auto mousePt = gui_win->mousePos();
	mrpt::math::TLine3D ray;
	vp->get3DRayForPixelCoord(mousePt.x(), mousePt.y(), ray);

	// Create a 3D plane, i.e. Z=0
	const auto ground_plane =
		mrpt::math::TPlane::From3Points({0, 0, 0}, {1, 0, 0}, {0, 1, 0});

	// Intersection of the line with the plane:
	mrpt::math::TObject3D inters;
	mrpt::math::intersect(ray, ground_plane, inters);

	// Interpret the intersection as a point, if there is an
	// intersection:
	if (inters.getPoint(clickedPt))
	{
		// Move object to the position picked by the user:
		// vp->getByClass<CDisk>(0)->setLocation(clickedPt);
	}

#if MRPT_VERSION >= 0x211
	const auto screen = gui_win->screen();
	const bool leftClick = screen->mouseState() == 0x01;

	// Replace object?
	if (btnReplaceObject && btnReplaceObject->pushed())
	{
		static bool isReplacing = false;

		// Start of replace? When the button push is released:
		if (!isReplacing && !leftClick)
		{
			isReplacing = true;
		}
		if (gui_selectedObject.simulable)
		{
			// btnReplaceObject->screen()->setCursor()

			mrpt::math::TPose3D p = gui_selectedObject.simulable->getPose();
			p.x = clickedPt.x;
			p.y = clickedPt.y;

			gui_selectedObject.simulable->setPose(p);
		}
		if (isReplacing && leftClick)
		{
			isReplacing = false;
			btnReplaceObject->setPushed(false);
		}
	}
#endif

	MRPT_END
}

void World::internal_process_pending_gui_user_tasks()
{
	guiUserPendingTasksMtx_.lock();

	for (const auto& task : guiUserPendingTasks_)
	{
		task();
	}
	guiUserPendingTasks_.clear();

	guiUserPendingTasksMtx_.unlock();
}

void World::internalRunSensorsOn3DScene(
	mrpt::opengl::COpenGLScene& physicalObjects)
{
	auto tle = mrpt::system::CTimeLoggerEntry(
		timlogger_, "internalRunSensorsOn3DScene");

	for (auto& v : vehicles_)
		for (auto& sensor : v.second->getSensors())
			if (sensor) sensor->simulateOn3DScene(physicalObjects);

	// clear the flag of pending 3D simulation required:
	clear_pending_running_sensors_on_3D_scene();
}

void World::internalUpdate3DSceneObjects(
	mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical)
{
	// Update view of map elements
	// -----------------------------
	auto tle =
		mrpt::system::CTimeLoggerEntry(timlogger_, "update_GUI.2.map-elements");

	for (auto& e : worldElements_) e->guiUpdate(viz, physical);

	tle.stop();

	// Update view of vehicles
	// -----------------------------
	timlogger_.enter("update_GUI.3.vehicles");

	for (auto& v : vehicles_) v.second->guiUpdate(viz, physical);

	timlogger_.leave("update_GUI.3.vehicles");

	// Update view of blocks
	// -----------------------------
	timlogger_.enter("update_GUI.4.blocks");

	for (auto& v : blocks_) v.second->guiUpdate(viz, physical);

	timlogger_.leave("update_GUI.4.blocks");

	// Other messages
	// -----------------------------
	timlogger_.enter("update_GUI.5.text-msgs");
	if (gui_.lbCpuUsage)
	{
		// 1st line: time
		double cpu_usage_ratio =
			std::max(1e-10, timlogger_.getMeanTime("run_simulation.cpu_dt")) /
			std::max(1e-10, timlogger_.getMeanTime("run_simulation.dt"));

		gui_.lbCpuUsage->setCaption(mrpt::format(
			"Time: %s (CPU usage: %.03f%%)",
			mrpt::system::formatTimeInterval(get_simul_time()).c_str(),
			cpu_usage_ratio * 100.0));

		// User supplied-lines:
		guiMsgLinesMtx_.lock();
		const std::string msg_lines = guiMsgLines_;
		guiMsgLinesMtx_.unlock();

		int nextStatusLine = 0;
		if (!msg_lines.empty())
		{
			// split lines:
			std::vector<std::string> lines;
			mrpt::system::tokenize(msg_lines, "\r\n", lines);
			for (const auto& l : lines)
				gui_.lbStatuses.at(nextStatusLine++)->setCaption(l);
		}
		gui_.lbStatuses.at(nextStatusLine++)
			->setCaption(std::string("Mouse: ") + gui_.clickedPt.asString());
	}

	timlogger_.leave("update_GUI.5.text-msgs");

	// Camera follow modes:
	// -----------------------
	if (!guiOptions_.follow_vehicle.empty())
	{
		if (auto it = vehicles_.find(guiOptions_.follow_vehicle);
			it != vehicles_.end())
		{
			const mrpt::poses::CPose2D pose = it->second->getCPose2D();
			gui_.gui_win->camera().setCameraPointing(pose.x(), pose.y(), 0.0f);
		}
		else
		{
			MRPT_LOG_THROTTLE_ERROR_FMT(
				5.0,
				"GUI: Camera set to follow vehicle named '%s' which can't be "
				"found!",
				guiOptions_.follow_vehicle.c_str());
		}
	}
}

void World::update_GUI(TUpdateGUIParams* guiparams)
{
	// First call?
	// -----------------------
	{
		auto lock = mrpt::lockHelper(gui_thread_start_mtx_);
		if (!gui_thread_running_ && !gui_thread_.joinable())
		{
			MRPT_LOG_DEBUG("[update_GUI] Launching GUI thread...");

			gui_thread_ = std::thread(&World::internal_GUI_thread, this);
#if MRPT_VERSION >= 0x204
			mrpt::system::thread_name("guiThread", gui_thread_);
#endif

			const int MVSIM_OPEN_GUI_TIMEOUT_MS =
				mrpt::get_env<int>("MVSIM_OPEN_GUI_TIMEOUT_MS", 3000);

			for (int timeout = 0; timeout < MVSIM_OPEN_GUI_TIMEOUT_MS / 10;
				 timeout++)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				if (gui_thread_running_) break;
			}

			if (!gui_thread_running_)
			{
				THROW_EXCEPTION("Timeout waiting for GUI to open!");
			}
			else
			{
				MRPT_LOG_DEBUG("[update_GUI] GUI thread started.");
			}
		}
	}

	if (!gui_.gui_win)
	{
		MRPT_LOG_THROTTLE_WARN(
			5.0,
			"[World::update_GUI] GUI window has been closed, but note that "
			"simulation keeps running.");
		return;
	}

	timlogger_.enter("update_GUI");	 // Don't count initialization, since that
									 // is a total outlier and lacks interest!

	guiMsgLinesMtx_.lock();
	guiMsgLines_ = guiparams->msg_lines;
	guiMsgLinesMtx_.unlock();

	timlogger_.leave("update_GUI");

	// Key-strokes:
	// -----------------------
	if (guiparams && lastKeyEventValid_)
	{
		auto lck = mrpt::lockHelper(lastKeyEventMtx_);

		guiparams->keyevent = std::move(lastKeyEvent_);
		lastKeyEventValid_ = false;
	}
}

// This method is ensured to be run in the GUI thread
void World::internal_gui_on_observation(
	const Simulable& veh, const mrpt::obs::CObservation::Ptr& obs)
{
	if (!obs) return;

	if (auto obs3D =
			std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(obs);
		obs3D)
	{
		internal_gui_on_observation_3Dscan(veh, obs3D);
	}
	else if (auto obsIm =
				 std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(obs);
			 obsIm)
	{
		internal_gui_on_observation_image(veh, obsIm);
	}
}

void World::internal_gui_on_observation_3Dscan(
	const Simulable& veh,
	const std::shared_ptr<mrpt::obs::CObservation3DRangeScan>& obs)
{
	using namespace std::string_literals;

	if (!gui_.gui_win || !obs) return;

	mrpt::math::TPoint2D rgbImageWinSize = {0, 0};

	if (obs->hasIntensityImage)
	{
		rgbImageWinSize = internal_gui_on_image(
			veh.getName() + "/"s + obs->sensorLabel + "_rgb"s,
			obs->intensityImage, 5);
	}
	if (obs->hasRangeImage)
	{
		mrpt::math::CMatrixFloat d;
		d = obs->rangeImage.asEigen().cast<float>() *
			(obs->rangeUnits / obs->maxRange);

		mrpt::img::CImage imDepth;
		imDepth.setFromMatrix(d, true /* in range [0,1] */);

		internal_gui_on_image(
			veh.getName() + "/"s + obs->sensorLabel + "_depth"s, imDepth,
			5 + 5 + rgbImageWinSize.x);
	}
}

void World::internal_gui_on_observation_image(
	const Simulable& veh,
	const std::shared_ptr<mrpt::obs::CObservationImage>& obs)
{
	using namespace std::string_literals;

	if (!gui_.gui_win || !obs || obs->image.isEmpty()) return;

	mrpt::math::TPoint2D rgbImageWinSize = {0, 0};

	rgbImageWinSize = internal_gui_on_image(
		veh.getName() + "/"s + obs->sensorLabel + "_rgb"s, obs->image, 5);
}

mrpt::math::TPoint2D World::internal_gui_on_image(
	const std::string& label, const mrpt::img::CImage& im, int winPosX)
{
	mrpt::gui::MRPT2NanoguiGLCanvas* glControl;

	// Once creation:
	if (!guiObsViz_.count(label))
	{
		auto& w = guiObsViz_[label] =
			gui_.gui_win->createManagedSubWindow(label);

		w->setLayout(new nanogui::GridLayout(
			nanogui::Orientation::Vertical, 1, nanogui::Alignment::Fill, 2, 2));

		// Guess window size:
		int winW = im.getWidth(), winH = im.getHeight();

		// Guess if we need to decimate subwindow size:
		while (winW >= 512 || winH >= 512)
		{
			winW /= 2;
			winH /= 2;
		}

		glControl = w->add<mrpt::gui::MRPT2NanoguiGLCanvas>();
		glControl->setSize({winW, winH});
		glControl->setFixedSize({winW, winH});

		static std::map<int, int> numGuiWindows;
		w->setPosition(
			{winPosX, 20 + (numGuiWindows[winPosX]++) * (winH + 10)});

		auto lck = mrpt::lockHelper(glControl->scene_mtx);

		glControl->scene = mrpt::opengl::COpenGLScene::Create();
		gui_.gui_win->performLayout();
	}

	// Update from sensor data:
	auto& w = guiObsViz_[label];

	glControl =
		dynamic_cast<mrpt::gui::MRPT2NanoguiGLCanvas*>(w->children().at(1));
	ASSERT_(glControl != nullptr);

	auto lck = mrpt::lockHelper(glControl->scene_mtx);
	glControl->scene->getViewport()->setImageView(im);

	return mrpt::math::TPoint2D(w->size().x(), w->size().y());
}

void World::internalGraphicsLoopTasksForSimulation()
{
	try
	{
		// Update all GUI elements:
		ASSERT_(worldVisual_);

		auto lckPhys = mrpt::lockHelper(physical_objects_mtx());

		internalUpdate3DSceneObjects(*worldVisual_, worldPhysical_);

		internalRunSensorsOn3DScene(worldPhysical_);

		lckPhys.unlock();

		// handle user custom 3D visual objects:
		{
			const auto lck = mrpt::lockHelper(guiUserObjectsMtx_);
			// replace list of smart pointers (fast):
			if (guiUserObjectsPhysical_)
				*glUserObjsPhysical_ = *guiUserObjectsPhysical_;
			if (guiUserObjectsViz_) *glUserObjsViz_ = *guiUserObjectsViz_;
		}
	}
	catch (const std::exception& e)
	{
		// In case of an exception in the functions above,
		// abort. Otherwise, the error may repeat over and over forever
		// and the main thread will never know about it.
		MRPT_LOG_ERROR(e.what());
		simulator_must_close(true);
	}
}

void World::setLightDirectionFromAzimuthElevation(
	const float azimuth, const float elevation)
{
	const mrpt::math::TPoint3Df dir = {
		-cos(azimuth) * cos(elevation), -sin(azimuth) * cos(elevation),
		-sin(elevation)};

	ASSERT_(worldVisual_);

	auto lckPhys = mrpt::lockHelper(physical_objects_mtx());

	auto vv = worldVisual_->getViewport();
	auto vp = worldPhysical_.getViewport();

	vv->lightParameters().direction = dir;
	vp->lightParameters().direction = dir;
}
