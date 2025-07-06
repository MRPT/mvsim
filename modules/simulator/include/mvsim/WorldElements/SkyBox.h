/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/opengl/CSetOfObjects.h>
#include <mvsim/WorldElements/WorldElementBase.h>

namespace mvsim
{
/** A skybox visual decoration.
 *
 * \note This element requires building against MRPT >=2.7.0
 *
 *  \ingroup world_elements_module
 */
class SkyBox : public WorldElementBase
{
	DECLARES_REGISTER_WORLD_ELEMENT(SkyBox)
   public:
	SkyBox(World* parent, const rapidxml::xml_node<char>* root);
	virtual ~SkyBox();

	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override;

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly) override;

	// It holds a CSkyBox object. Stored as base CRenderizable to prevent
	// depending in this public header on mrpt >=2.7.0 so mvsim can still be
	// built with older mrpt versions.
	mrpt::opengl::CRenderizable::Ptr glSkyBox_;

	mrpt::opengl::CRenderizable::Ptr glSkyBoxPrepared_;
};
}  // namespace mvsim
