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
#include <Shapes/Composition.h>
#include <Shapes/Box.h>
#include <Shapes/LineSegment.h>
#include <Shapes/Point.h>
#include <Shapes/Shape.h>
#include <Shapes/ShapeGroup.h>
#include <Shapes/Sphere.h>

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
        void globalPhysicsStep(Float delta);
        void physicsStep(Float time, Float delta);
        void applyForce(const Vector2& position, const Vector2& force);
        void applyImpulse(const Vector2& position, const Vector2& impulse);

        DebugTools::ResourceManager debugResourceManager;

        Timeline timeline;
        Scene2D scene;
        SceneGraph::Camera2D *camera;
        SceneGraph::DrawableGroup2D drawables;
        Shapes::ShapeGroup2D visualizationShapes, collisionShapes;

        Object2D *tube, *vehicle, *body, *armLeft, *armRight, *engineLeft, *engineRight;

        struct {
            Shapes::Shape<Shapes::Sphere2D> *tube;
            Shapes::Shape<Shapes::Composition2D>* vehicle;
        } shapes;

        struct {
            Vector2 weight, engineLeftArm, engineRightArm;
            Vector2 spring[3];
            Vector2 friction[3];
        } forces;

        struct {
            Float massInverted,
                powerArm,
                restitution,
                staticFriction, dynamicFriction,
                springConstant,
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
    Configuration conf;
    #ifndef CORRADE_TARGET_NACL
    conf.setTitle("Kotel::Prototype::Forces2D");
    #endif
    conf.setSampleCount(16);
    if(!tryCreateContext(conf)) {
        Warning() << "Cannot enable 16x MSAA, fallback to no-AA rendering";
        createContext(conf.setSampleCount(0));
    }

    Renderer::setClearColor(Color3(0.125f));
    Renderer::setFeature(Renderer::Feature::Blending, true);
    Renderer::setBlendFunction(Renderer::BlendFunction::SourceAlpha, Renderer::BlendFunction::OneMinusSourceAlpha);

    /* Camera setup */
    (camera = new SceneGraph::Camera2D(scene))
        ->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
        .setProjection(Vector2(3.0f));

    /* Debug draw setup */
    auto primaryForcesOptions = new DebugTools::ForceRendererOptions;
    primaryForcesOptions->setSize(0.00025f).setColor(Color4::fromHSV(Deg(190.0f), 0.75f, 0.9f, 0.5f));
    auto springOptions = new DebugTools::ForceRendererOptions;
    springOptions->setSize(0.000025f).setColor(Color4(1.0f, 1.0f));
    auto collisionOptions = new DebugTools::ShapeRendererOptions;
    collisionOptions->setPointSize(0.1f).setColor(Color4::fromHSV(Deg(25.0f), 0.75f, 0.9f, 0.75f));
    auto vehicleOptions = new DebugTools::ShapeRendererOptions;
    vehicleOptions->setColor(Color3(0.5f));
    auto parametersOptions = new DebugTools::ObjectRendererOptions;
    parametersOptions->setSize(0.1f);
    debugResourceManager.set("primaryForces", primaryForcesOptions)
        .set("spring", springOptions)
        .set("collision", collisionOptions)
        .set("vehicle", vehicleOptions)
        .set("parameters", parametersOptions);

    /* Parameters */
    parameters.physicsTimeDelta = 1.0f/120.0f;
    parameters.gravity = Vector2::yAxis(-9.81f);
    parameters.armAngle = Deg(110.0f);
    parameters.engineAngle = Deg(0.0f);
    parameters.armRadius = 1.0f;
    const Float massBody = 260.0f;
    const Float massArm = 80.0f;
    parameters.massInverted = 1.0f/(massBody + 2*massArm);
    parameters.powerArm = 1500.0f;
    parameters.restitution = 0.6f;
    parameters.staticFriction = 0.16f;
    parameters.dynamicFriction = 0.1f;
    parameters.springConstant = 500000; /** @todo Is this in sane range? */

    /* Object initialization */
    tube = new Object2D(&scene);
    (vehicle = new Object2D(&scene))
        ->translate(Vector2::yAxis(0.3f));
    (body = new Object2D(vehicle))
        ->translate({});
    (armLeft = new Object2D(vehicle))
        ->translate(Vector2::yAxis(-parameters.armRadius))
        .rotate(-parameters.armAngle/2);
    (armRight = new Object2D(vehicle))
        ->translate(Vector2::yAxis(-parameters.armRadius))
        .rotate(parameters.armAngle/2);
    (engineLeft = new Object2D(vehicle))
        ->rotate(-parameters.engineAngle)
        .translate(Vector2::yAxis(-parameters.armRadius))
        .rotate(-parameters.armAngle/2);
    (engineRight = new Object2D(vehicle))
        ->rotate(parameters.engineAngle)
        .translate(Vector2::yAxis(-parameters.armRadius))
        .rotate(parameters.armAngle/2);

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

    /* Tube collision shapes */
    shapes.tube = new Shapes::Shape<Shapes::Sphere2D>(*tube, {{}, 1.0f}, &collisionShapes);
    new DebugTools::ShapeRenderer2D(*shapes.tube, "collision", &drawables);

    /* Vehicle collision shapes */
    shapes.vehicle = new Shapes::Shape<Shapes::Composition2D>(*vehicle,
        Shapes::Point2D(body->transformation().translation()+Vector2::yAxis(0.15f)) ||
        Shapes::Point2D(engineLeft->transformation().translation()) ||
        Shapes::Point2D(engineRight->transformation().translation()), &collisionShapes);
    new DebugTools::ShapeRenderer2D(*shapes.vehicle, "collision", &drawables);

    /* Vehicle visualization */
    auto bodyShape = new Shapes::Shape<Shapes::Sphere2D>(*body, {{}, 0.2f}, &visualizationShapes);
    const Shapes::LineSegment2D arm(Vector2::yAxis(parameters.armRadius-0.2f), Vector2::yAxis(0.03f));
    auto armLeftShape = new Shapes::Shape<Shapes::LineSegment2D>(*armLeft, arm, &visualizationShapes);
    auto armRightShape = new Shapes::Shape<Shapes::LineSegment2D>(*armRight, arm, &visualizationShapes);
    const Shapes::Box2D engine(Matrix3::scaling({0.1f, 0.03f}));
    auto engineLeftShape = new Shapes::Shape<Shapes::Box2D>(*engineLeft, engine, &visualizationShapes);
    auto engineRightShape = new Shapes::Shape<Shapes::Box2D>(*engineRight, engine, &visualizationShapes);
    new DebugTools::ShapeRenderer2D(*bodyShape, "vehicle", &drawables);
    new DebugTools::ShapeRenderer2D(*armLeftShape, "vehicle", &drawables);
    new DebugTools::ShapeRenderer2D(*armRightShape, "vehicle", &drawables);
    new DebugTools::ShapeRenderer2D(*engineLeftShape, "vehicle", &drawables);
    new DebugTools::ShapeRenderer2D(*engineRightShape, "vehicle", &drawables);

    /* Vehicle center-of-mass visualization */
    new DebugTools::ObjectRenderer2D(*vehicle, "parameters", &drawables);

    /* Gravity force visualization */
    new DebugTools::ForceRenderer2D(*vehicle, {}, forces.weight, "primaryForces", &drawables);

    /* Engine, friction and spring forces visualization */
    new DebugTools::ForceRenderer2D(*engineLeft, {}, forces.engineLeftArm, "primaryForces", &drawables);
    new DebugTools::ForceRenderer2D(*engineRight, {}, forces.engineRightArm, "primaryForces", &drawables);
    for(std::size_t i = 0; i != 3; ++i) {
        const Vector2 position = shapes.vehicle->shape().get<Shapes::Point2D>(i).position();
        new DebugTools::ForceRenderer2D(*vehicle, position, forces.spring[i], "spring", &drawables);
    }

    /* Zero-time physics step */
    timeline.start();
}

void Forces2D::viewportEvent(const Vector2i& size) {
    defaultFramebuffer.setViewport({{}, size});

    camera->setViewport(size);
}

void Forces2D::drawEvent() {
    defaultFramebuffer.bind(FramebufferTarget::Draw);
    defaultFramebuffer.clear(FramebufferClear::Color);

    /* Do physics steps in elapsed time */
    globalPhysicsStep(timeline.previousFrameDuration());

    visualizationShapes.setClean();
    camera->draw(drawables);

    swapBuffers();
    timeline.nextFrame();
    redraw();
}

void Forces2D::keyPressEvent(KeyEvent& event) {
    if(event.key() == KeyEvent::Key::Left) {
        state.currentPowerLeftArm = {};
        state.currentPowerRightArm = parameters.powerArm;
    } else if(event.key() == KeyEvent::Key::Right) {
        state.currentPowerLeftArm = parameters.powerArm;
        state.currentPowerRightArm = {};
    } else if(event.key() == KeyEvent::Key::R) {
        vehicle->resetTransformation().translate(Vector2::yAxis(0.3f));
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

void Forces2D::globalPhysicsStep(const Float delta) {
    state.physicsTimeAccumulator += delta;

    while(state.physicsTimeAccumulator >= parameters.physicsTimeDelta) {
        /* Compute force and torque at original position */
        state.force = {};
        state.torque = {};
        physicsStep(state.physicsTime, parameters.physicsTimeDelta);

        state.linearVelocity += 0.5f*state.force*parameters.massInverted*parameters.physicsTimeDelta;
        state.angularSpeed += 0.5f*state.torque*parameters.momentOfInertiaInverted*parameters.physicsTimeDelta;

        /* Check count of penetrations */
        UnsignedInt penetrationCount = 0;
        const Shapes::Point2D* penetrations[3];
        for(std::size_t i = 0; i != 3; ++i) {
            const auto* p = &shapes.vehicle->transformedShape().get<Shapes::Point2D>(i);
            if(!(*p % shapes.tube->transformedShape()))
                penetrations[penetrationCount++] = p;
        }

        /* Collision response */
        Vector2 normal;
        switch(penetrationCount) {
            /* No collisions, nothing to do */
            case 0: break;

            /* One or two collision, compute proper normal and position */
            case 2:
                normal -= penetrations[1]->position();
            case 1: {
                normal -= penetrations[0]->position();
                const Vector2 absolutePosition = normal/penetrationCount;
                const Vector2 position = absolutePosition - vehicle->absoluteTransformation().translation();
                const Vector2 velocity = state.linearVelocity + state.angularSpeed*position.perpendicular();

                /* Collision impulse in direction of the normal, don't allow it
                   to go in wrong direction (i.e. when the spring forces are
                   already doing the collision response)e */
                const Float collisionImpulseMagnitude = Math::max(0.0f,
                    -(parameters.restitution + 1.0f)*Vector2::dot(velocity, normal)/(
                        parameters.massInverted*normal.dot() +
                        parameters.momentOfInertiaInverted*Math::pow<2>(Vector2::cross(position, normal))
                    ));
                const Vector2 collisionImpulse = collisionImpulseMagnitude*normal;

                /* Neither normal nor tangent are normalized, the impulses are
                   computed in a way to avoid any need for that */
                const Vector2 tangent = normal.perpendicular()*Math::sign(Vector2::dot(velocity, normal.perpendicular()));

                /* Friction impulse in direction of tangent */
                const Float frictionImpulseMagnitude = -Vector2::dot(velocity, tangent)/(
                    parameters.massInverted*tangent.dot() +
                    parameters.momentOfInertiaInverted*Math::pow<2>(Vector2::cross(position, tangent))
                );
                const Vector2 frictionImpulse = Math::abs(frictionImpulseMagnitude) < collisionImpulse.length()*parameters.staticFriction ?
                    frictionImpulseMagnitude*tangent : -collisionImpulse.length()*tangent*parameters.dynamicFriction;

                /* Apply impulses */
                applyImpulse(absolutePosition, frictionImpulse);
                applyImpulse(absolutePosition, collisionImpulse);

            }; break;

            /* Three collisions shouldn't happen */
            default: CORRADE_ASSERT(false, "The vehicle escaped the known universe!", );
        }

        /* New position and rotation (around COM) */
        vehicle->translate(state.linearVelocity*parameters.physicsTimeDelta)
            .rotate(Rad(state.angularSpeed*parameters.physicsTimeDelta), SceneGraph::TransformationType::Local)
            .normalizeRotation();

        /* Compute force at new position */
        state.force = {};
        state.torque = {};
        physicsStep(state.physicsTime+parameters.physicsTimeDelta, parameters.physicsTimeDelta);

        /* New velocity */
        state.linearVelocity += 0.5f*state.force*parameters.massInverted*parameters.physicsTimeDelta;
        state.angularSpeed += 0.5f*state.torque*parameters.momentOfInertiaInverted*parameters.physicsTimeDelta;

        state.physicsTimeAccumulator -= parameters.physicsTimeDelta;
        state.physicsTime += parameters.physicsTimeDelta;
    }
}

void Forces2D::physicsStep(const Float, const Float) {
    /* Reset forces */
    forces.engineLeftArm = forces.engineRightArm =
        forces.friction[0] = forces.friction[1] = forces.friction[2] =
            forces.spring[0] = forces.spring[1] = forces.spring[2] = {};

    /* Get engine force direction from transformations */
    const auto engineDirectionLeft = Vector2(engineLeft->absoluteTransformation().rotation());
    const auto engineDirectionRight = Vector2(engineRight->absoluteTransformation().rotation());

    /* Add engine forces */
    forces.engineLeftArm = engineDirectionLeft*state.currentPowerLeftArm;
    forces.engineRightArm = -engineDirectionRight*state.currentPowerRightArm;

    /* Apply forces */
    applyForce(vehicle->absoluteTransformation().translation(), forces.weight);
    applyForce(engineLeft->absoluteTransformation().translation(), forces.engineLeftArm);
    applyForce(engineRight->absoluteTransformation().translation(), forces.engineRightArm);

    /* Apply spring forces for all penetrations */
    collisionShapes.setClean();
    for(std::size_t i = 0; i != 3; ++i) {
        const auto& p = shapes.vehicle->transformedShape().get<Shapes::Point2D>(i);
        if(p % shapes.tube->transformedShape()) continue;

        /* Spring force is proportional to penetration depth */
        forces.spring[i] = p.position()*(parameters.springConstant*
            (shapes.tube->shape().radius()*p.position().lengthInverted() - 1.0f));
        applyForce(p.position(), forces.spring[i]);
    }
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
