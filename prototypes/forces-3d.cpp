/*
    This file is part of Kotel.

    Copyright © 2013 Vladimír Vondruš <mosra@centrum.cz>

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

#include <numeric>
#include <DefaultFramebuffer.h>
#include <Renderer.h>
#ifndef CORRADE_TARGET_NACL
#include <Platform/Sdl2Application.h>
#else
#include <Platform/NaClApplication.h>
#endif
#include <Primitives/Cylinder.h>
#include <MeshTools/CompressIndices.h>
#include <MeshTools/Duplicate.h>
#include <MeshTools/FlipNormals.h>
#include <MeshTools/Interleave.h>
#include <MeshTools/Transform.h>
#include <SceneGraph/Camera3D.h>
#include <SceneGraph/Drawable.h>
#include <SceneGraph/DualQuaternionTransformation.h>
#include <SceneGraph/Scene.h>
#include <Shaders/MeshVisualizer.h>
#include <Trade/MeshData3D.h>

#ifdef MAGNUM_BUILD_STATIC
#include <Shaders/magnumShadersResourceImport.hpp>
#endif

#include "Kotel.h"

namespace Kotel { namespace Prototype {

typedef SceneGraph::Object<SceneGraph::DualQuaternionTransformation> Object3D;
typedef SceneGraph::Scene<SceneGraph::DualQuaternionTransformation> Scene3D;

class Forces3D: public Platform::Application {
    public:
        explicit Forces3D(const Arguments& arguments);

        void viewportEvent(const Vector2i& size) override;
        void drawEvent() override;

    private:
        Scene3D scene;
        Object3D* cameraObject;
        SceneGraph::Camera3D* camera;
        SceneGraph::DrawableGroup3D drawables;
};

Forces3D::Forces3D(const Arguments& arguments): Platform::Application(arguments, nullptr) {
    /* Try to create MSAA context */
    Configuration conf;
    #ifndef CORRADE_TARGET_NACL
    conf.setTitle("Kotel::Prototype::Forces3D");
    #endif
    conf.setSampleCount(16);
    if(!tryCreateContext(conf)) {
        Warning() << "Cannot enable 16x MSAA, fallback to no-AA rendering";
        createContext(conf.setSampleCount(0));
    }

    Renderer::setFeature(Renderer::Feature::DepthTest, true);
    Renderer::setFeature(Renderer::Feature::FaceCulling, true);
    Renderer::setClearColor(Color3(0.125f));

    (cameraObject = new Object3D(&scene))
        ->translate(Vector3::zAxis(5.0f));
    (camera = new SceneGraph::Camera3D(*cameraObject))
        ->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
        .setPerspective(Deg(35.0f), 4.0f/3, 0.001f, 100.0f);

    class Tube: public Object3D, SceneGraph::Drawable3D {
        public:
            Tube(Object3D* parent, SceneGraph::DrawableGroup3D* drawables): Object3D(parent), SceneGraph::Drawable3D(*this, drawables), shader(Shaders::MeshVisualizer::Flag::Wireframe|Shaders::MeshVisualizer::Flag::NoGeometryShader) {
                Trade::MeshData3D cube = Primitives::Cylinder::solid(10, 16, 10.0f);
                std::vector<Float> vertexIndices(cube.indices().size(), 0.0f);
                std::iota(vertexIndices.begin(), vertexIndices.end(), 0.0f);
                MeshTools::flipFaceWinding(cube.indices());
                MeshTools::transformVectorsInPlace(Quaternion::rotation(Deg(90.0f), Vector3::xAxis()), cube.positions(0));
                MeshTools::interleave(mesh, vertexBuffer, Buffer::Usage::StaticDraw,
                    MeshTools::duplicate(cube.indices(), cube.positions(0)),
                    vertexIndices);

                mesh.setPrimitive(cube.primitive())
                    .addInterleavedVertexBuffer(vertexBuffer, 0,
                        Shaders::MeshVisualizer::Position(), Shaders::MeshVisualizer::VertexIndex());
            }

        private:
            void draw(const Matrix4& transformationMatrix, SceneGraph::AbstractCamera3D& camera) override {
                shader.setTransformationProjectionMatrix(camera.projectionMatrix()*transformationMatrix)
                    .setViewportSize(Vector2(camera.viewport()))
                    .setColor(Color3(0.15f))
                    .setWireframeColor(Color4::fromHSV(Deg(25.0f), 0.75f, 0.9f, 0.75f))
                    .use();

                mesh.draw();
            }

            Buffer indexBuffer, vertexBuffer;
            Mesh mesh;
            Shaders::MeshVisualizer shader;
    };

    new Tube(&scene, &drawables);
}

void Forces3D::viewportEvent(const Vector2i& size) {
    defaultFramebuffer.setViewport({{}, size});

    camera->setViewport(size);
}

void Forces3D::drawEvent() {
    defaultFramebuffer.clear(FramebufferClear::Color|FramebufferClear::Depth);

    camera->draw(drawables);

    swapBuffers();
}

}}

MAGNUM_APPLICATION_MAIN(Kotel::Prototype::Forces3D)
