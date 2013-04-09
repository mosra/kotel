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
        ~Forces2D();

        void viewportEvent(const Vector2i& size) override;
        void drawEvent() override;
        void keyPressEvent(KeyEvent& event) override;
        void keyReleaseEvent(KeyEvent& event) override;

    private:
        void globalPhysicsStep(const Float time, const Float delta);
        void physicsStep(const Float time, const Float delta);
        void applyForce(const Vector2& position, const Vector2& force);
        void applyImpulse(const Vector2& position, const Vector2& impulse);

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
            Vector2 weight,
                engineLeftArm, engineRightArm,
                frictionLeftArm, frictionRightArm,
                totalLeftArm, totalRightArm;
        } forces;

        struct {
            Float massInverted,
                powerArm,
                restitution,
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
    debugResourceManager.set("weight", (new DebugTools::ForceRendererOptions)
        ->setSize(0.00025f)->setColor(Color4<>::fromHSV(Deg(190.0f), 0.75f, 0.9f, 0.5f)));
    debugResourceManager.set("engines", (new DebugTools::ForceRendererOptions)
        ->setSize(0.00025f)->setColor(Color4<>::fromHSV(Deg(50.0f), 0.75f, 0.9f, 0.5f)));
    debugResourceManager.set("friction", (new DebugTools::ForceRendererOptions)
        ->setSize(0.00025f)->setColor(Color4<>::fromHSV(Deg(115.0f), 0.75f, 0.9f, 0.5f)));
    debugResourceManager.set("total", (new DebugTools::ForceRendererOptions)
        ->setSize(0.00025f)->setColor(Color4<>::fromHSV(Deg(245.0f), 0.75f, 0.9f, 0.75f)));
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
    parameters.powerArm = 500.0f;
    parameters.restitution = 0.6f;
    parameters.friction = 0.16f;

    /* Object initialization */
    tube = new Object2D(&scene);
    (vehicle = new Object2D(&scene))
        ->translate(Vector2::yAxis(0.3f));
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
    body->translate(-centerOfMass);
    armLeft->translate(-centerOfMass);
    armRight->translate(-centerOfMass);
    engineLeft->translate(-centerOfMass);
    engineRight->translate(-centerOfMass);

    /* Weight force */
    forces.weight = parameters.gravity/parameters.massInverted;

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

    /* Gravity force visualization */
    new DebugTools::ForceRenderer2D(vehicle, {}, &forces.weight, "weight", &drawables);

    /* Engine and friction forces visualization */
    new DebugTools::ForceRenderer2D(engineLeft, {}, &forces.engineLeftArm, "engines", &drawables);
    new DebugTools::ForceRenderer2D(engineRight, {}, &forces.engineRightArm, "engines", &drawables);
    new DebugTools::ForceRenderer2D(engineLeft, {}, &forces.frictionLeftArm, "friction", &drawables);
    new DebugTools::ForceRenderer2D(engineRight, {}, &forces.frictionRightArm, "friction", &drawables);

    /* Total forces visualization */
    new DebugTools::ForceRenderer2D(engineLeft, {}, &forces.totalLeftArm, "total", &drawables);
    new DebugTools::ForceRenderer2D(engineRight, {}, &forces.totalRightArm, "total", &drawables);

    /* Zero-time physics step */
    physicsStep(state.physicsTime, parameters.physicsTimeDelta);
    timeline.start();
}

Forces2D::~Forces2D() {
    /* Delete things which are not garbage collected by SceneGraph */
    delete shapes.tubeMin;
    delete shapes.tubeMax;
    delete shapes.vehicleBody;
    delete shapes.vehicleArmLeft;
    delete shapes.vehicleArmRight;
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
        state.currentPowerLeftArm = -parameters.powerArm;
        state.currentPowerRightArm = -parameters.powerArm;
    } else if(event.key() == KeyEvent::Key::Right) {
        state.currentPowerLeftArm = parameters.powerArm;
        state.currentPowerRightArm = parameters.powerArm;
    } else if(event.key() == KeyEvent::Key::R) {
        vehicle->resetTransformation()->translate(Vector2::yAxis(0.3f));
        state.currentPowerLeftArm = state.currentPowerRightArm =
            state.angularSpeed = {};
        state.linearVelocity = {};
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

    /* Check count of penetrations */
    UnsignedInt penetrationCount = 0;
    const Physics::Point2D* penetrations[3];
    collisionShapes.setClean();
    const Physics::Point2D* const penetrationCandidates[3] = {shapes.vehicleBody, shapes.vehicleArmLeft, shapes.vehicleArmRight};
    for(std::size_t i = 0; i != 3; ++i) if(!(*penetrationCandidates[i] % *shapes.tubeMax))
        penetrations[penetrationCount++] = penetrationCandidates[i];

    /* Collision response */
    Vector2 normal;
    switch(penetrationCount) {
        /* No collisions, nothing to do */
        case 0: break;

        /* One or two collision, compute proper normal and position */
        case 2:
            normal -= penetrations[1]->transformedPosition();
        case 1: {
            normal -= penetrations[0]->transformedPosition();
            const Vector2 absolutePosition = normal/penetrationCount;
            const Vector2 position = absolutePosition - vehicle->absoluteTransformation().translation();
            const Vector2 velocity = state.linearVelocity + state.angularSpeed*position.perpendicular();
            const Vector2 impulse = (-(1.0f + parameters.restitution)*Vector2::dot(velocity, normal)/
                (normal.dot()*parameters.massInverted + Math::pow<2>(Vector2::cross(position, normal))*parameters.momentOfInertiaInverted))*normal;

            applyImpulse(absolutePosition, impulse);
        }; break;

        /* Three collisions shouldn't happen */
        default: CORRADE_ASSERT(false, "The vehicle escaped the known universe!", );
    }

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
    const Vector2 engineDirectionLeft(engineLeft->absoluteTransformation().rotation());
    const Vector2 engineDirectionRight(engineRight->absoluteTransformation().rotation());

    /* Reset forces */
    forces.totalLeftArm = forces.totalRightArm =
        forces.frictionLeftArm = forces.frictionRightArm = {};

    /* Add engine forces */
    forces.engineLeftArm = engineDirectionLeft*state.currentPowerLeftArm;
    forces.engineRightArm = engineDirectionRight*state.currentPowerRightArm;
    forces.totalLeftArm += forces.engineLeftArm;
    forces.totalRightArm += forces.engineRightArm;

    /* Apply forces */
    applyForce(vehicle->absoluteTransformation().translation(), forces.weight);
    applyForce(engineLeft->absoluteTransformation().translation(), forces.totalLeftArm);
    applyForce(engineLeft->absoluteTransformation().translation(), forces.totalRightArm);
}

void Forces2D::applyForce(const Vector2& position, const Vector2& force) {
    state.force += force;
    state.torque += Vector2::cross(position - vehicle->absoluteTransformation().translation(), force);
}

void Forces2D::applyImpulse(const Vector2& position, const Vector2& impulse) {
    state.linearVelocity += impulse*parameters.massInverted;
    state.angularSpeed += Vector2::cross(position - vehicle->absoluteTransformation().translation(), impulse)*parameters.momentOfInertiaInverted;
}

}}

MAGNUM_APPLICATION_MAIN(Kotel::Prototype::Forces2D)
