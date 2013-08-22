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
#include <DebugTools/ShapeRenderer.h>
#include <DebugTools/ResourceManager.h>
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
#include <Shapes/Capsule.h>
#include <Shapes/Composition.h>
#include <Shapes/LineSegment.h>
#include <Shapes/Shape.h>
#include <Shapes/ShapeGroup.h>
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
        DebugTools::ResourceManager debugResourceManager;

        Scene3D scene;
        Object3D* cameraObject;
        SceneGraph::Camera3D* camera;
        SceneGraph::DrawableGroup3D drawables;
        Shapes::ShapeGroup3D visualizationShapes;

        struct {
            Deg armAngle, engineAngle;
            Float armRadius;
        } parameters;

        Object3D *vehicle, *body, *armLeft, *armRight, *engineLeft, *engineRight;
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

    /* Camera setup */
    (cameraObject = new Object3D(&scene))
        ->translate(Vector3::zAxis(5.0f));
    (camera = new SceneGraph::Camera3D(*cameraObject))
        ->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
        .setPerspective(Deg(35.0f), 4.0f/3, 0.001f, 100.0f);

    /* Debug draw setup */
    debugResourceManager.set("vehicle", DebugTools::ShapeRendererOptions().setColor(Color3(0.5f)));

    /* Parameters */
    parameters.armAngle = Deg(110.0f);
    parameters.engineAngle = Deg(0.0f);
    parameters.armRadius = 1.0f;

    /* Object initialization */
    (vehicle = new Object3D(&scene))
        ->translate(Vector3::yAxis(0.3f));
    (body = new Object3D(vehicle))
        ->translate({});
    (armLeft = new Object3D(vehicle))
        ->translate(Vector3::yAxis(-parameters.armRadius))
        .rotateZ(-parameters.armAngle/2);
    (armRight = new Object3D(vehicle))
        ->translate(Vector3::yAxis(-parameters.armRadius))
        .rotateZ(parameters.armAngle/2);
    (engineLeft = new Object3D(vehicle))
        ->rotateZ(-parameters.engineAngle)
        .translate(Vector3::yAxis(-parameters.armRadius))
        .rotateZ(-parameters.armAngle/2);
    (engineRight = new Object3D(vehicle))
        ->rotateZ(parameters.engineAngle)
        .translate(Vector3::yAxis(-parameters.armRadius))
        .rotateZ(parameters.armAngle/2);

    /* Vehicle visualization */
    auto bodyShape = new Shapes::Shape<Shapes::Capsule3D>(*body, {Vector3::zAxis(0.5f), Vector3::zAxis(-0.5f), 0.2f}, &visualizationShapes);
    const auto arm = Shapes::LineSegment3D(Vector3::yAxis(parameters.armRadius-0.2f), {0.0f, 0.05f, 0.45f}) ||
                     Shapes::LineSegment3D(Vector3::yAxis(parameters.armRadius-0.2f), {0.0f, 0.05f, -0.65f});
    auto armLeftShape = new Shapes::Shape<Shapes::Composition3D>(*armLeft, arm, &visualizationShapes);
    auto armRightShape = new Shapes::Shape<Shapes::Composition3D>(*armRight, arm, &visualizationShapes);
    const Shapes::Capsule3D engine(Vector3::zAxis(0.55f), Vector3::zAxis(-0.75f), 0.05f);
    auto engineLeftShape = new Shapes::Shape<Shapes::Capsule3D>(*engineLeft, engine, &visualizationShapes);
    auto engineRightShape = new Shapes::Shape<Shapes::Capsule3D>(*engineRight, engine, &visualizationShapes);
    new DebugTools::ShapeRenderer3D(*bodyShape, "vehicle", &drawables);
    new DebugTools::ShapeRenderer3D(*armLeftShape, "vehicle", &drawables);
    new DebugTools::ShapeRenderer3D(*armRightShape, "vehicle", &drawables);
    new DebugTools::ShapeRenderer3D(*engineLeftShape, "vehicle", &drawables);
    new DebugTools::ShapeRenderer3D(*engineRightShape, "vehicle", &drawables);

    /* Tube visualization */
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

    visualizationShapes.setClean();
    camera->draw(drawables);

    swapBuffers();
}

}}

MAGNUM_APPLICATION_MAIN(Kotel::Prototype::Forces3D)
