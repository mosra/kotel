#include <Math/Functions.h>
#include <DefaultFramebuffer.h>
#include <Renderer.h>
#include <Timeline.h>
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
#include <Physics/LineSegment.h>
#include <Physics/Point.h>
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
        void physicsStep(const Float time, const Float delta);

        DebugTools::ResourceManager debugResourceManager;

        Timeline timeline;
        Scene2D scene;
        Object2D cameraObject;
        SceneGraph::Camera2D<> *camera;
        SceneGraph::DrawableGroup<2> drawables;
        Physics::ObjectShapeGroup2D shapes;

        Object2D *tube, *vehicle;

        struct {
            Vector2 weightBody,
                weightLeftArm, weightRightArm,
                engineLeftArm, engineRightArm,
                frictionLeftArm, frictionRightArm,
                totalNormalLeftArm, totalNormalRightArm,
                totalTangentLeftArm, totalTangentRightArm;
        } forces;

        struct {
            Float massBody,
                massArm,
                powerArm,
                friction;

            Float physicsTimeDelta;

            Vector2 gravity,
                centerOfMass;

            Deg armAngle;
            DualComplex baseLeftArmTransformation,
                baseRightArmTransformation;
        } parameters;

        struct {
            Float currentPowerLeftArm,
                currentPowerRightArm,
                physicsTime,
                physicsTimeAccumulator;
        } state;

};

Forces2D::Forces2D(const Arguments& arguments): Platform::Application(arguments, nullptr), state{{}, {}, {}, {}} {
    /* Try to create MSAA context */
    auto conf = new Configuration();
    #ifndef CORRADE_TARGET_NACL
    conf->setTitle("Kotel::Prototype::Forces2D");
    #endif
    conf->setSampleCount(16);
    if(!tryCreateContext(conf)) {
        Warning() << "Cannot enable 16x MSAA, fallback to no-AA rendering";
        createContext(conf->setSampleCount(0));
    } else delete conf;

    Renderer::setClearColor(Color3<>(0.125f));
    Renderer::setFeature(Renderer::Feature::Blending, true);
    Renderer::setBlendFunction(Renderer::BlendFunction::SourceAlpha, Renderer::BlendFunction::OneMinusSourceAlpha);

    /* Parameters */
    parameters.physicsTimeDelta = 1.0f/120.0f;
    parameters.gravity = Vector2::yAxis(-9.81);
    parameters.armAngle = Deg(110.0f);
    parameters.massBody = 260.0f;
    parameters.massArm = 80.0f;
    parameters.powerArm = 1000.0f;
    parameters.friction = 0.16f;

    /* Camera setup */
    cameraObject.setParent(&scene);
    (camera = new SceneGraph::Camera2D<>(&cameraObject))
        ->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
        ->setProjection(Vector2(3.0f));

    /* Debug draw setup */
    debugResourceManager.set("gravity", (new DebugTools::ForceRendererOptions)
        ->setSize(0.0005f)->setColor(Color4<>::fromHSV(Deg(190.0f), 0.75f, 0.9f, 0.5f)));
    debugResourceManager.set("engines", (new DebugTools::ForceRendererOptions)
        ->setSize(0.0005f)->setColor(Color4<>::fromHSV(Deg(50.0f), 0.75f, 0.9f, 0.5f)));
    debugResourceManager.set("friction", (new DebugTools::ForceRendererOptions)
        ->setSize(0.0005f)->setColor(Color4<>::fromHSV(Deg(115.0f), 0.75f, 0.9f, 0.5f)));
    debugResourceManager.set("tangent", (new DebugTools::ForceRendererOptions)
        ->setSize(0.0005f)->setColor(Color4<>::fromHSV(Deg(245.0f), 0.75f, 0.9f, 0.75f)));
    debugResourceManager.set("normal", (new DebugTools::ForceRendererOptions)
        ->setSize(0.0005f)->setColor(Color4<>::fromHSV(Deg(5.0f), 0.75f, 0.9f, 0.75f)));
    debugResourceManager.set("tube", (new DebugTools::ShapeRendererOptions)
        ->setColor(Color3<>(0.2f)));
    debugResourceManager.set("vehicle", (new DebugTools::ShapeRendererOptions)
        ->setColor(Color3<>(0.5f)));
    debugResourceManager.set("parameters", (new DebugTools::ShapeRendererOptions)
        ->setPointSize(0.1f)->setColor(Color3<>::fromHSV(Deg(25.0f), 0.75f, 0.9f)));

    /* Tube */
    tube = new Object2D(&scene);
    auto tubeShape = new Physics::ObjectShape2D(tube, &shapes);
    tubeShape->setShape(
        Physics::Sphere2D({}, 1.05f) ||
        Physics::Sphere2D({}, 1.15f)
    );
    new DebugTools::ShapeRenderer2D(tubeShape, "tube", &drawables);

    /* Vehicle parameters */
    parameters.baseLeftArmTransformation = DualComplex::rotation(-parameters.armAngle/2)*DualComplex::translation(Vector2::yAxis(-1.0f));
    parameters.baseRightArmTransformation = DualComplex::rotation(parameters.armAngle/2)*DualComplex::translation(Vector2::yAxis(-1.0f));
    parameters.centerOfMass = ((parameters.baseLeftArmTransformation.translation() +
        parameters.baseRightArmTransformation.translation())*
        parameters.massArm + Vector2()*parameters.massBody)/(parameters.massArm*2 + parameters.massBody);

    /* Vehicle */
    vehicle = new Object2D(&scene);
    const auto armA = Vector2::yAxis(-0.200f);
    const auto armB = Vector2::yAxis(-0.970f);
    const auto leftArmRotation = Complex::rotation(-parameters.armAngle/2);
    const auto rightArmRotation = Complex::rotation(parameters.armAngle/2);
    auto vehicleShape = new Physics::ObjectShape2D(vehicle, &shapes);
    vehicleShape->setShape(
        Physics::Sphere2D({}, .2f) ||
        Physics::LineSegment2D(leftArmRotation.transformVector(armA), leftArmRotation.transformVector(armB)) ||
        Physics::LineSegment2D(rightArmRotation.transformVector(armA), rightArmRotation.transformVector(armB)) ||
        Physics::Box2D(parameters.baseLeftArmTransformation.toMatrix()*Matrix3::scaling({0.1f, 0.03f})) ||
        Physics::Box2D(parameters.baseRightArmTransformation.toMatrix()*Matrix3::scaling({0.1f, 0.03f}))
    );
    new DebugTools::ShapeRenderer2D(vehicleShape, "vehicle", &drawables);

    /* Vehicle center-of-mass */
    auto vehicleCenterOfMassShape = new Physics::ObjectShape2D(vehicle, &shapes);
    vehicleCenterOfMassShape->setShape(Physics::Point2D(parameters.centerOfMass));
    new DebugTools::ShapeRenderer2D(vehicleCenterOfMassShape, "parameters", &drawables);

    /* Gravity forces */
    forces.weightBody = parameters.gravity*parameters.massBody;
    forces.weightLeftArm = parameters.gravity*parameters.massArm;
    forces.weightRightArm = parameters.gravity*parameters.massArm;
    new DebugTools::ForceRenderer2D(vehicle, {},
        &forces.weightBody, "gravity", &drawables);
    new DebugTools::ForceRenderer2D(vehicle, parameters.baseLeftArmTransformation.translation(),
        &forces.weightLeftArm, "gravity", &drawables);
    new DebugTools::ForceRenderer2D(vehicle, parameters.baseRightArmTransformation.translation(),
        &forces.weightRightArm, "gravity", &drawables);

    /* Engine and friction forces */
    new DebugTools::ForceRenderer2D(vehicle, parameters.baseLeftArmTransformation.translation(),
        &forces.engineLeftArm, "engines", &drawables);
    new DebugTools::ForceRenderer2D(vehicle, parameters.baseRightArmTransformation.translation(),
        &forces.engineRightArm, "engines", &drawables);
    new DebugTools::ForceRenderer2D(vehicle, parameters.baseLeftArmTransformation.translation(),
        &forces.frictionLeftArm, "friction", &drawables);
    new DebugTools::ForceRenderer2D(vehicle, parameters.baseRightArmTransformation.translation(),
        &forces.frictionRightArm, "friction", &drawables);

    /* Tangent and normal forces */
    new DebugTools::ForceRenderer2D(vehicle, parameters.baseLeftArmTransformation.translation(),
        &forces.totalTangentLeftArm, "tangent", &drawables);
    new DebugTools::ForceRenderer2D(vehicle, parameters.baseRightArmTransformation.translation(),
        &forces.totalTangentRightArm, "tangent", &drawables);
    new DebugTools::ForceRenderer2D(vehicle, parameters.baseLeftArmTransformation.translation(),
        &forces.totalNormalLeftArm, "normal", &drawables);
    new DebugTools::ForceRenderer2D(vehicle, parameters.baseRightArmTransformation.translation(),
        &forces.totalNormalRightArm, "normal", &drawables);

    timeline.start();
}

void Forces2D::viewportEvent(const Vector2i& size) {
    defaultFramebuffer.setViewport({{}, size});

    camera->setViewport(size);
}

void Forces2D::drawEvent() {
    defaultFramebuffer.bind(DefaultFramebuffer::Target::Draw);
    defaultFramebuffer.clear(DefaultFramebuffer::Clear::Color);

    /* Do physics steps in elapsed time */
    state.physicsTimeAccumulator += timeline.previousFrameDuration();
    while(state.physicsTimeAccumulator >= parameters.physicsTimeDelta) {
        physicsStep(state.physicsTime, parameters.physicsTimeDelta);
        state.physicsTimeAccumulator -= parameters.physicsTimeDelta;
        state.physicsTime += parameters.physicsTimeDelta;
    }

    shapes.setClean();
    camera->draw(drawables);

    swapBuffers();
    timeline.nextFrame();
}

void Forces2D::keyPressEvent(KeyEvent& event) {
    if(event.key() == KeyEvent::Key::Left) {
        state.currentPowerLeftArm = parameters.powerArm;
        state.currentPowerRightArm = parameters.powerArm;
    } else if(event.key() == KeyEvent::Key::Right) {
        state.currentPowerLeftArm = -parameters.powerArm;
        state.currentPowerRightArm = -parameters.powerArm;
    } else if(event.key() == KeyEvent::Key::Up) {
        vehicle->rotate(Deg(5.0f))->normalizeRotation();
    } else if(event.key() == KeyEvent::Key::Down) {
        vehicle->rotate(Deg(-5.0f))->normalizeRotation();
    } else return;

    event.setAccepted();
    redraw();
}

void Forces2D::keyReleaseEvent(KeyEvent& event) {
    state.currentPowerLeftArm = state.currentPowerRightArm = {};

    event.setAccepted();
    redraw();
}

void Forces2D::physicsStep(const Float, const Float) {
    /* Compute tangent and normal vectors */
    const Vector2 tangentLeftArm = (vehicle->transformation().rotation()*
        parameters.baseLeftArmTransformation.rotation()).transformVector(Vector2::xAxis());
    const Vector2 tangentRightArm = (vehicle->transformation().rotation()*
        parameters.baseRightArmTransformation.rotation()).transformVector(Vector2::xAxis());
    const Vector2 normalLeftArm = (vehicle->transformation().rotation()*
        parameters.baseLeftArmTransformation.rotation()).transformVector(-Vector2::yAxis());
    const Vector2 normalRightArm = (vehicle->transformation().rotation()*
        parameters.baseRightArmTransformation.rotation()).transformVector(-Vector2::yAxis());

    /* Reset */
    forces.totalNormalLeftArm = forces.totalNormalRightArm =
        forces.totalTangentLeftArm = forces.totalTangentRightArm = {};

    /* Propagate body weight to arms */
    forces.totalNormalLeftArm += forces.weightBody.projectedOntoNormalized(normalLeftArm);
    forces.totalNormalRightArm += forces.weightBody.projectedOntoNormalized(normalRightArm);

    /* Decompose arm weight to normal and tangent */
    forces.totalNormalLeftArm += forces.weightLeftArm.projectedOntoNormalized(normalLeftArm);
    forces.totalNormalRightArm += forces.weightRightArm.projectedOntoNormalized(normalRightArm);
    forces.totalTangentLeftArm += forces.weightLeftArm.projectedOntoNormalized(tangentLeftArm);
    forces.totalTangentRightArm += forces.weightRightArm.projectedOntoNormalized(tangentRightArm);

    /* Add engine forces to tangent */
    forces.engineLeftArm = tangentLeftArm*state.currentPowerLeftArm;
    forces.engineRightArm = tangentRightArm*state.currentPowerRightArm;
    forces.totalTangentLeftArm += forces.engineLeftArm;
    forces.totalTangentRightArm += forces.engineRightArm;

    /* Add friction forces to tangent */
    forces.frictionLeftArm = -Math::sign(forces.totalTangentLeftArm)*Math::min(
        Math::abs(tangentLeftArm*forces.totalNormalLeftArm.length()*parameters.friction),
        Math::abs(forces.totalTangentLeftArm));
    forces.frictionRightArm = -Math::sign(forces.totalTangentRightArm)*Math::min(
        Math::abs(tangentRightArm*forces.totalNormalRightArm.length()*parameters.friction),
        Math::abs(forces.totalTangentRightArm));
    forces.totalTangentLeftArm += forces.frictionLeftArm;
    forces.totalTangentRightArm += forces.frictionRightArm;
}

}}

MAGNUM_APPLICATION_MAIN(Kotel::Prototype::Forces2D)
