/*
Copyright 2021 Siyuan Pan <pansiyuan.cs@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/
#include <ajisai/factory/factory.h>
#include <ajisai/factory/creator/scene_creators.h>
#include <ajisai/factory/creator/primitive_creators.h>
#include <ajisai/factory/creator/geometry_creators.h>
#include <ajisai/factory/creator/material_creators.h>
#include <ajisai/factory/creator/texture2d_creators.h>
#include <ajisai/factory/creator/aggregate_creators.h>
#include <ajisai/factory/creator/camera_creators.h>
#include <ajisai/factory/creator/renderer_creators.h>
#include <ajisai/factory/creator/post_processor_creators.h>
#include <ajisai/factory/creator/medium_creators.h>

AJ_BEGIN

CreateFactory::CreateFactory()
    : factory_tuple_{Factory<Scene>("scene"),
                     Factory<Camera>("camera"),
                     Factory<Renderer>("renderer"),
                     Factory<PostProcessor>("post_processor"),
                     Factory<Aggregate>("aggregate"),
                     Factory<Primitive>("primitive"),
                     Factory<Geometry>("geometry"),
                     Factory<Material>("material"),
                     Factory<Medium>("medium"),
                     Factory<Texture2D>("texture2D")} {
  AddSceneFactory(GetFactory<Scene>());
  AddCameraFactory(GetFactory<Camera>());
  AddRendererFactory(GetFactory<Renderer>());
  AddPostProcessorFactory(GetFactory<PostProcessor>());
  AddAggregateFactory(GetFactory<Aggregate>());
  AddPrimitiveFactory(GetFactory<Primitive>());
  AddGeometricFactory(GetFactory<Geometry>());
  AddMaterialFactory(GetFactory<Material>());
  AddMediumFactory(GetFactory<Medium>());
  AddTexture2DFactory(GetFactory<Texture2D>());
}

AJ_END