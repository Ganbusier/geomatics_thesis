// Copyright (c) 2016  GeometryFactory Sarl (France)
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Three/include/CGAL/Three/Scene_item_with_properties.h $
// $Id: include/CGAL/Three/Scene_item_with_properties.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Maxime GIMENO

#ifndef SCENE_ITEM_WITH_PROPERTIES_H
#define SCENE_ITEM_WITH_PROPERTIES_H

#include <CGAL/license/Three.h>
#include <QtGlobal>
#ifdef demo_framework_EXPORTS
#  define DEMO_FRAMEWORK_EXPORT Q_DECL_EXPORT
#else
#  define DEMO_FRAMEWORK_EXPORT Q_DECL_IMPORT
#endif

namespace CGAL
{
namespace Three {
  class Scene_item;

//! Base class to allow an item to copy properties from another.
//! Properties represent the current state of an item : its color,
//! the position of its manipulated frame, ...
class DEMO_FRAMEWORK_EXPORT Scene_item_with_properties {
public:
  virtual ~Scene_item_with_properties();
 //!\brief Copies properties from another Scene_item.
 //!
 //! Override this function to specify what must be copied.
 virtual void copyProperties(Scene_item*)=0; //pure virtual method
};
}
}
#endif // SCENE_ITEM_WITH_PROPERTIES_H

