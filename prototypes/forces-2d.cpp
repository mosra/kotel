#include <Renderer.h>
#include <DefaultFramebuffer.h>
#include <DebugTools/ForceRenderer.h>
#include <DebugTools/ResourceManager.h>
#include <DebugTools/ShapeRenderer.h>
#ifndef CORRADE_TARGET_NACL
#include <Platform/Sdl2Application.h>
#else
#include <Platform/NaClApplication.h>
#endif
#include <SceneGraph/Camera2D.h>
#include <SceneGraph/DualComplexTransformation.h>
#include <SceneGraph/Object.h>
#include <SceneGraph/Scene.h>
#include <Physics/ObjectShape.h>
#include <Physics/ObjectShapeGroup.h>
#include <Physics/Box.h>
#include <Physics/ShapeGroup.h>
#include <Physics/Sphere.h>

#ifdef MAGNUM_BUILD_STATIC
#include <Shaders/magnumShadersResourceImport.hpp>
#endif

#include "Kotel.h"

namespace Kotel { namespace Prototype {

class Forces2D: public Platform::Application {
    public:
        explicit Forces2D(const Arguments& arguments);

        void viewportEvent(const Vector2i& size) override;
        void drawEvent() override;
        void keyPressEvent(KeyEvent& event) override;
        void keyReleaseEvent(KeyEvent& event) override;

    private:
        DebugTools::ResourceManager debugResourceManager;

        Scene2D scene;
        Object2D cameraObject;
        SceneGraph::Camera2D<> *camera;
        SceneGraph::DrawableGroup<2> drawables;
        Physics::ObjectShapeGroup2D shapes;

        Object2D *tube, *vehicle;

        Vector2 gravity;

        struct {
            Vector2 weightBody,
                weightLeftArm, weightRightArm,
                engineLeftArm, engineRightArm;
        } forces;

        struct {
            Float body,
                arms;
        } masses;

        struct {
            Float arms;
        } powers;

        DualComplex baseLeftArmTransformation, baseRightArmTransformation;
};

Forces2D::Forces2D(const Arguments& arguments): Platform::Application(arguments, (new Configuration())
    #ifndef CORRADE_TARGET_NACL
    ->setTitle("Kotel::Prototype::Forces2D")
    ->setSampleCount(16)
    #endif
) {
    Renderer::setClearColor(Color3<>(0.125f));

    /* Parameters */
    gravity = Vector2::yAxis(-9.81);
    masses.body = 260.0f;
    masses.arms = 80.0f;
    powers.arms = 1000.0f;

    /* Camera setup */
    cameraObject.setParent(&scene);
    (camera = new SceneGraph::Camera2D<>(&cameraObject))
        ->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
        ->setProjection(Vector2(3.0f));

    /* Debug draw setup */
    debugResourceManager.set("gravity", (new DebugTools::ForceRendererOptions())
        ->setSize(0.0005f)->setColor(Color3<>::fromHSV(Deg(206.0f), 0.75f, 0.9f)));
    debugResourceManager.set("engines", (new DebugTools::ForceRendererOptions())
        ->setSize(0.0005f)->setColor(Color3<>::fromHSV(Deg(50.0f), 0.75f, 0.9f)));
    debugResourceManager.set("tube", (new DebugTools::ShapeRendererOptions())
        ->setColor(Color3<>(0.2f)));
    debugResourceManager.set("vehicle", (new DebugTools::ShapeRendererOptions())
        ->setColor(Color3<>(0.5f)));

    /* Tube */
    tube = new Object2D(&scene);
    auto tubeShape = new Physics::ObjectShape2D(tube, &shapes);
    tubeShape->setShape(
        Physics::Sphere2D({}, 1.05f) ||
        Physics::Sphere2D({}, 1.15f)
    );
    new DebugTools::ShapeRenderer2D(tubeShape, "tube", &drawables);

    /* Vehicle */
    baseLeftArmTransformation = DualComplex::rotation(Deg(-35.0f))*DualComplex::translation(Vector2::xAxis(1.0f));
    baseRightArmTransformation = DualComplex::rotation(Deg(35.0f))*DualComplex::translation(Vector2::xAxis(-1.0f));
    vehicle = new Object2D(&scene);
    auto vehicleShape = new Physics::ObjectShape2D(vehicle, &shapes);
    vehicleShape->setShape(
        Physics::Sphere2D({}, .2f) ||
        Physics::Box2D(Matrix3::rotation(Deg(-35.0f))*Matrix3::translation(Vector2::xAxis(.585f))*Matrix3::scaling({.385f, .02f})) ||
        Physics::Box2D(Matrix3::rotation(Deg(35.0f))*Matrix3::translation(Vector2::xAxis(-.585f))*Matrix3::scaling({.385f, .02f})) ||
        Physics::Box2D(baseLeftArmTransformation.toMatrix()*Matrix3::scaling({.03f, .1f})) ||
        Physics::Box2D(baseRightArmTransformation.toMatrix()*Matrix3::scaling({.03f, .1f}))
    );
    new DebugTools::ShapeRenderer2D(vehicleShape, "vehicle", &drawables);

    /* Gravity forces */
    forces.weightBody = gravity*masses.body;
    forces.weightLeftArm = gravity*masses.arms;
    forces.weightRightArm = gravity*masses.arms;
    new DebugTools::ForceRenderer2D(vehicle, {},
        &forces.weightBody, "gravity", &drawables);
    new DebugTools::ForceRenderer2D(vehicle, baseLeftArmTransformation.translation(),
        &forces.weightLeftArm, "gravity", &drawables);
    new DebugTools::ForceRenderer2D(vehicle, baseRightArmTransformation.translation(),
        &forces.weightRightArm, "gravity", &drawables);

    /* Engine forces */
    new DebugTools::ForceRenderer2D(vehicle, baseLeftArmTransformation.translation(),
        &forces.engineLeftArm, "engines", &drawables);
    new DebugTools::ForceRenderer2D(vehicle, baseRightArmTransformation.translation(),
        &forces.engineRightArm, "engines", &drawables);
}

void Forces2D::viewportEvent(const Vector2i& size) {
    defaultFramebuffer.setViewport({{}, size});

    camera->setViewport(size);
}

void Forces2D::drawEvent() {
    defaultFramebuffer.bind(DefaultFramebuffer::Target::Draw);
    defaultFramebuffer.clear(DefaultFramebuffer::Clear::Color);

    shapes.setClean();
    camera->draw(drawables);

    swapBuffers();
}

void Forces2D::keyPressEvent(KeyEvent& event) {
    if(event.key() == KeyEvent::Key::Left || event.key() == KeyEvent::Key::Right) {
        const auto force = Vector2::yAxis(event.key() == KeyEvent::Key::Left ? powers.arms : -powers.arms);
        forces.engineLeftArm = (vehicle->transformation().rotation()*baseLeftArmTransformation.rotation())
            .transformVector(force);
        forces.engineRightArm = (vehicle->transformation().rotation()*baseRightArmTransformation.rotation())
            .transformVector(-force);
    } else return;

    event.setAccepted();
    redraw();
}

void Forces2D::keyReleaseEvent(KeyEvent& event) {
    forces.engineLeftArm = {};
    forces.engineRightArm = {};

    event.setAccepted();
    redraw();
}

}}

MAGNUM_APPLICATION_MAIN(Kotel::Prototype::Forces2D)
