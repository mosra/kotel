#include <Math/Functions.h>
#include <DefaultFramebuffer.h>
#include <Renderer.h>
#include <Timeline.h>
#include <DebugTools/ForceRenderer.h>
#include <DebugTools/ObjectRenderer.h>
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
        void globalPhysicsStep(const Float time, const Float delta);
        void physicsStep(const Float time, const Float delta);
        void applyForce(const Vector2& position, const Vector2& force);

        DebugTools::ResourceManager debugResourceManager;

        Timeline timeline;
        Scene2D scene;
        Object2D cameraObject;
        SceneGraph::Camera2D<> *camera;
        SceneGraph::DrawableGroup<2> drawables;
        Physics::ObjectShapeGroup2D visualizationShapes, collisionShapes;

        Object2D *tube, *vehicle, *body, *armLeft, *armRight, *engineLeft, *engineRight;

        struct {
            Physics::Sphere2D *tubeMin, *tubeMax;
            Physics::Point2D *vehicleBody, *vehicleArmLeft, *vehicleArmRight;
        } shapes;

        struct {
            Vector2 weightBody,
                weightLeftArm, weightRightArm,
                engineLeftArm, engineRightArm,
                frictionLeftArm, frictionRightArm,
                totalNormalLeftArm, totalNormalRightArm,
                totalTangentLeftArm, totalTangentRightArm;
        } forces;

        struct {
            Float massInverted,
                powerArm,
                friction,
                momentOfInertiaInverted;

            Float physicsTimeDelta;
            Vector2 gravity;

            Deg armAngle, engineAngle;
            Float armRadius;
        } parameters;

        struct {
            Float currentPowerLeftArm,
                currentPowerRightArm,
                physicsTime,
                physicsTimeAccumulator,
                angularSpeed,
                torque;

            Vector2 linearVelocity,
                force;
        } state;

};

Forces2D::Forces2D(const Arguments& arguments): Platform::Application(arguments, nullptr), state() {
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
    debugResourceManager.set("collision", (new DebugTools::ShapeRendererOptions)
        ->setPointSize(0.1f)->setColor(Color4<>::fromHSV(Deg(25.0f), 0.75f, 0.9f, 0.75f)));
    debugResourceManager.set("vehicle", (new DebugTools::ShapeRendererOptions)
        ->setColor(Color3<>(0.5f)));
    debugResourceManager.set("parameters", (new DebugTools::ObjectRendererOptions)
        ->setSize(0.1f));

    /* Parameters */
    parameters.physicsTimeDelta = 1.0f/120.0f;
    parameters.gravity = Vector2::yAxis(-9.81);
    parameters.armAngle = Deg(110.0f);
    parameters.engineAngle = Deg(0.0f);
    parameters.armRadius = 1.0f;
    const Float massBody = 260.0f;
    const Float massArm = 80.0f;
    parameters.massInverted = 1.0f/(massBody + 2*massArm);
    parameters.powerArm = 1000.0f;
    parameters.friction = 0.16f;

    /* Object initialization */
    tube = new Object2D(&scene);
    vehicle = new Object2D(&scene);
    (body = new Object2D(vehicle))
        ->translate({});
    (armLeft = new Object2D(vehicle))
        ->translate(Vector2::yAxis(-parameters.armRadius))
        ->rotate(-parameters.armAngle/2);
    (armRight = new Object2D(vehicle))
        ->translate(Vector2::yAxis(-parameters.armRadius))
        ->rotate(parameters.armAngle/2);
    (engineLeft = new Object2D(vehicle))
        ->rotate(-parameters.engineAngle)
        ->translate(Vector2::yAxis(-parameters.armRadius))
        ->rotate(-parameters.armAngle/2);
    (engineRight = new Object2D(vehicle))
        ->rotate(parameters.engineAngle)
        ->translate(Vector2::yAxis(-parameters.armRadius))
        ->rotate(parameters.armAngle/2);

    /* Compute center of mass and move it to center of the object */
    const Vector2 centerOfMass = (body->transformation().translation()*massBody +
        (engineLeft->transformation().translation() + engineRight->transformation().translation())*
        massArm)*parameters.massInverted;
    vehicle->translate(centerOfMass);
    body->translate(-centerOfMass);
    armLeft->translate(-centerOfMass);
    armRight->translate(-centerOfMass);
    engineLeft->translate(-centerOfMass);
    engineRight->translate(-centerOfMass);

    /* Weight forces */
    forces.weightBody = parameters.gravity*parameters.massBody;
    forces.weightLeftArm = parameters.gravity*parameters.massArm;
    forces.weightRightArm = parameters.gravity*parameters.massArm;

    /* Moment of inertia */
    parameters.momentOfInertiaInverted = 1.0f/(body->transformation().translation().dot()*massBody +
        engineLeft->transformation().translation().dot()*massArm +
        engineRight->transformation().translation().dot()*massArm);

    /* Tube and vehicle collision shapes */
    shapes.tubeMin = new Physics::Sphere2D({}, 0.99f);
    shapes.tubeMax = new Physics::Sphere2D({}, 1.01f);
    shapes.vehicleBody = new Physics::Point2D(body->transformation().translation()+Vector2::yAxis(0.15f));
    shapes.vehicleArmLeft = new Physics::Point2D(engineLeft->transformation().translation());
    shapes.vehicleArmRight = new Physics::Point2D(engineRight->transformation().translation());
    new DebugTools::ShapeRenderer2D((new Physics::ObjectShape2D(tube, &collisionShapes))
        ->setShape(std::ref(*shapes.tubeMin) || std::ref(*shapes.tubeMax)),
        "collision", &drawables);
    new DebugTools::ShapeRenderer2D((new Physics::ObjectShape2D(vehicle, &collisionShapes))
        ->setShape(std::ref(*shapes.vehicleBody) || std::ref(*shapes.vehicleArmLeft) || std::ref(*shapes.vehicleArmRight)),
        "collision", &drawables);

    /* Vehicle visualization */
    const auto armA = Vector2::yAxis(parameters.armRadius-0.2f);
    const auto armB = Vector2::yAxis(0.03f);
    new DebugTools::ShapeRenderer2D((new Physics::ObjectShape2D(body, &visualizationShapes))
        ->setShape(Physics::Sphere2D({}, 0.2f)), "vehicle", &drawables);
    new DebugTools::ShapeRenderer2D((new Physics::ObjectShape2D(armLeft, &visualizationShapes))
        ->setShape(Physics::LineSegment2D(armA, armB)), "vehicle", &drawables);
    new DebugTools::ShapeRenderer2D((new Physics::ObjectShape2D(armRight, &visualizationShapes))
        ->setShape(Physics::LineSegment2D(armA, armB)), "vehicle", &drawables);
    new DebugTools::ShapeRenderer2D((new Physics::ObjectShape2D(engineLeft, &visualizationShapes))
        ->setShape(Physics::Box2D(Matrix3::scaling({0.1f, 0.03f}))), "vehicle", &drawables);
    new DebugTools::ShapeRenderer2D((new Physics::ObjectShape2D(engineRight, &visualizationShapes))
        ->setShape(Physics::Box2D(Matrix3::scaling({0.1f, 0.03f}))), "vehicle", &drawables);

    /* Vehicle center-of-mass visualization */
    new DebugTools::ObjectRenderer2D(vehicle, "parameters", &drawables);

    /* Gravity forces visualization */
    new DebugTools::ForceRenderer2D(body, {}, &forces.weightBody, "gravity", &drawables);
    new DebugTools::ForceRenderer2D(engineLeft, {}, &forces.weightLeftArm, "gravity", &drawables);
    new DebugTools::ForceRenderer2D(engineRight, {}, &forces.weightRightArm, "gravity", &drawables);

    /* Engine and friction forces visualization */
    new DebugTools::ForceRenderer2D(engineLeft, {}, &forces.engineLeftArm, "engines", &drawables);
    new DebugTools::ForceRenderer2D(engineRight, {}, &forces.engineRightArm, "engines", &drawables);
    new DebugTools::ForceRenderer2D(engineLeft, {}, &forces.frictionLeftArm, "friction", &drawables);
    new DebugTools::ForceRenderer2D(engineRight, {}, &forces.frictionRightArm, "friction", &drawables);

    /* Tangent and normal forces visualization */
    new DebugTools::ForceRenderer2D(engineLeft, {}, &forces.totalTangentLeftArm, "tangent", &drawables);
    new DebugTools::ForceRenderer2D(engineRight, {}, &forces.totalTangentRightArm, "tangent", &drawables);
    new DebugTools::ForceRenderer2D(engineLeft, {}, &forces.totalNormalLeftArm, "normal", &drawables);
    new DebugTools::ForceRenderer2D(engineRight, {}, &forces.totalNormalRightArm, "normal", &drawables);

    /* Zero-time physics step */
    physicsStep(state.physicsTime, parameters.physicsTimeDelta);
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
        globalPhysicsStep(state.physicsTime, parameters.physicsTimeDelta);
        state.physicsTimeAccumulator -= parameters.physicsTimeDelta;
        state.physicsTime += parameters.physicsTimeDelta;
    }

    visualizationShapes.setClean();
    collisionShapes.setClean();
    camera->draw(drawables);

    swapBuffers();
    timeline.nextFrame();
    redraw();
}

void Forces2D::keyPressEvent(KeyEvent& event) {
    if(event.key() == KeyEvent::Key::Left) {
        state.currentPowerLeftArm = parameters.powerArm;
        state.currentPowerRightArm = parameters.powerArm;
    } else if(event.key() == KeyEvent::Key::Right) {
        state.currentPowerLeftArm = -parameters.powerArm;
        state.currentPowerRightArm = -parameters.powerArm;
    } else if(event.key() == KeyEvent::Key::Up) {
        vehicle->rotate(Deg(5.0f), SceneGraph::TransformationType::Local)->normalizeRotation();
    } else if(event.key() == KeyEvent::Key::Down) {
        vehicle->rotate(Deg(-5.0f), SceneGraph::TransformationType::Local)->normalizeRotation();
    } else return;

    event.setAccepted();
    redraw();
}

void Forces2D::keyReleaseEvent(KeyEvent& event) {
    state.currentPowerLeftArm = state.currentPowerRightArm = {};

    event.setAccepted();
    redraw();
}

void Forces2D::globalPhysicsStep(const Float time, const Float delta) {
    /* Compute force and torque at original position */
    state.force = {};
    state.torque = {};
    physicsStep(time, delta);

    state.linearVelocity += 0.5f*state.force*parameters.massInverted*delta;
    state.angularSpeed += 0.5f*state.torque*parameters.momentOfInertiaInverted*delta;

    /* New position and rotation (around COM) */
    vehicle->translate(state.linearVelocity*delta)
        ->rotate(Rad(state.angularSpeed*delta), SceneGraph::TransformationType::Local)
        ->normalizeRotation();

    /* Compute force at new position */
    state.force = {};
    state.torque = {};
    physicsStep(time+delta, delta);

    /* New velocity */
    state.linearVelocity += 0.5f*state.force*parameters.massInverted*delta;
    state.angularSpeed += 0.5f*state.torque*parameters.momentOfInertiaInverted*delta;
}

void Forces2D::physicsStep(const Float, const Float) {
    /* Compute tangent and normal vectors */
    const Vector2 engineLeftArm(engineLeft->absoluteTransformation().rotation());
    const Vector2 engineRightArm(engineRight->absoluteTransformation().rotation());
    const Vector2 tangentLeftArm(armLeft->absoluteTransformation().rotation());
    const Vector2 tangentRightArm(armRight->absoluteTransformation().rotation());
    const Vector2 normalLeftArm = tangentLeftArm.perpendicular();
    const Vector2 normalRightArm = tangentRightArm.perpendicular();

    /* Reset forces */
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

    /* Add engine forces*/
    forces.engineLeftArm = engineLeftArm*state.currentPowerLeftArm;
    forces.engineRightArm = engineRightArm*state.currentPowerRightArm;
    forces.totalTangentLeftArm += forces.engineLeftArm.projectedOntoNormalized(tangentLeftArm);
    forces.totalTangentRightArm += forces.engineRightArm.projectedOntoNormalized(tangentRightArm);
    forces.totalNormalLeftArm += forces.engineLeftArm.projectedOntoNormalized(normalLeftArm);
    forces.totalNormalRightArm += forces.engineRightArm.projectedOntoNormalized(normalRightArm);

    /* Add friction forces to tangent */
    forces.frictionLeftArm = -Math::sign(forces.totalTangentLeftArm)*Math::min(
        Math::abs(tangentLeftArm*forces.totalNormalLeftArm.length()*parameters.friction),
        Math::abs(forces.totalTangentLeftArm));
    forces.frictionRightArm = -Math::sign(forces.totalTangentRightArm)*Math::min(
        Math::abs(tangentRightArm*forces.totalNormalRightArm.length()*parameters.friction),
        Math::abs(forces.totalTangentRightArm));
    forces.totalTangentLeftArm += forces.frictionLeftArm;
    forces.totalTangentRightArm += forces.frictionRightArm;

    /* Ignore normal force, apply tangent ones */
    applyForce(engineLeft->absoluteTransformation().translation(), forces.totalTangentLeftArm);
    applyForce(engineLeft->absoluteTransformation().translation(), forces.totalTangentRightArm);
}

void Forces2D::applyForce(const Vector2& position, const Vector2& force) {
    state.force += force;
    state.torque += Vector2::cross(position - vehicle->absoluteTransformation().translation(), force);
}

}}

MAGNUM_APPLICATION_MAIN(Kotel::Prototype::Forces2D)
