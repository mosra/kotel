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
        Object2D cameraObject;
        SceneGraph::Camera2D<> *camera;
        SceneGraph::DrawableGroup<2> drawables;
        Shapes::ShapeGroup2D visualizationShapes, collisionShapes;

        Object2D *tube, *vehicle, *body, *armLeft, *armRight, *engineLeft, *engineRight;

        struct {
            Shapes::Shape<Shapes::Sphere2D>* tubeMin;
            Shapes::Shape<Shapes::Composition2D> *tubeMax, *vehicle;
        } shapes;

        struct {
            Vector2 weight,
                engineLeftArm, engineRightArm,
                frictionLeftArm, frictionRightArm,
                totalLeftArm, totalRightArm;

            Vector2 spring[3];
        } forces;

        struct {
            Float massInverted,
                powerArm,
                restitution,
                friction,
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
    debugResourceManager.set("spring", (new DebugTools::ForceRendererOptions)
        ->setSize(0.000025f)->setColor(Color4<>(1.0f, 1.0f)));
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
    parameters.springConstant = 1000000; /** @todo Is this in sane range? */

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

    /* Tube collision shapes */
    shapes.tubeMin = new Shapes::Shape<Shapes::Sphere2D>(tube, {{}, 0.99f}, &collisionShapes);
    new DebugTools::ShapeRenderer2D(shapes.tubeMin, "collision", &drawables);
    shapes.tubeMax = new Shapes::Shape<Shapes::Composition2D>(tube, !Shapes::Sphere2D({}, 1.01f), &collisionShapes);
    new DebugTools::ShapeRenderer2D(shapes.tubeMax, "collision", &drawables);

    /* Vehicle collision shapes */
    shapes.vehicle = new Shapes::Shape<Shapes::Composition2D>(vehicle,
        Shapes::Point2D(body->transformation().translation()+Vector2::yAxis(0.15f)) ||
        Shapes::Point2D(engineLeft->transformation().translation()) ||
        Shapes::Point2D(engineRight->transformation().translation()), &collisionShapes);
    new DebugTools::ShapeRenderer2D(shapes.vehicle, "collision", &drawables);

    /* Vehicle visualization */
    new DebugTools::ShapeRenderer2D(new Shapes::Shape<Shapes::Sphere2D>(body,
        {{}, 0.2f}, &visualizationShapes), "vehicle", &drawables);
    const Shapes::LineSegment2D arm(Vector2::yAxis(parameters.armRadius-0.2f), Vector2::yAxis(0.03f));
    new DebugTools::ShapeRenderer2D(new Shapes::Shape<Shapes::LineSegment2D>(armLeft,
        arm, &visualizationShapes), "vehicle", &drawables);
    new DebugTools::ShapeRenderer2D(new Shapes::Shape<Shapes::LineSegment2D>(armRight,
        arm, &visualizationShapes), "vehicle", &drawables);
    const Shapes::Box2D engine(Matrix3::scaling({0.1f, 0.03f}));
    new DebugTools::ShapeRenderer2D(new Shapes::Shape<Shapes::Box2D>(engineLeft,
        engine, &visualizationShapes), "vehicle", &drawables);
    new DebugTools::ShapeRenderer2D(new Shapes::Shape<Shapes::Box2D>(engineRight,
        engine, &visualizationShapes), "vehicle", &drawables);

    /* Vehicle center-of-mass visualization */
    new DebugTools::ObjectRenderer2D(vehicle, "parameters", &drawables);

    /* Gravity force visualization */
    new DebugTools::ForceRenderer2D(vehicle, {}, &forces.weight, "weight", &drawables);

    /* Engine, friction and spring forces visualization */
    new DebugTools::ForceRenderer2D(engineLeft, {}, &forces.engineLeftArm, "engines", &drawables);
    new DebugTools::ForceRenderer2D(engineRight, {}, &forces.engineRightArm, "engines", &drawables);
    new DebugTools::ForceRenderer2D(engineLeft, {}, &forces.frictionLeftArm, "friction", &drawables);
    new DebugTools::ForceRenderer2D(engineRight, {}, &forces.frictionRightArm, "friction", &drawables);
    for(std::size_t i = 0; i != 3; ++i)
        new DebugTools::ForceRenderer2D(vehicle, shapes.vehicle->shape().get<Shapes::Point2D>(i).position(),
            &forces.spring[i], "spring", &drawables);

    /* Total forces visualization */
    new DebugTools::ForceRenderer2D(engineLeft, {}, &forces.totalLeftArm, "total", &drawables);
    new DebugTools::ForceRenderer2D(engineRight, {}, &forces.totalRightArm, "total", &drawables);

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
            if(!(*p % shapes.tubeMin->transformedShape()))
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

                /* Impulse in direction of the normal, don't allow it to go in
                   wrong direction (i.e. when the spring forces are already
                   doing the collision response */
                const Vector2 impulse = Math::max(0.0f,
                    -(parameters.restitution + 1.0f)*Vector2::dot(velocity, normal)/(
                        parameters.massInverted*normal.dot() +
                        parameters.momentOfInertiaInverted*Math::pow<2>(Vector2::cross(position, normal))
                    ))*normal;

                applyImpulse(absolutePosition, impulse);
            }; break;

            /* Three collisions shouldn't happen */
            default: CORRADE_ASSERT(false, "The vehicle escaped the known universe!", );
        }

        /* New position and rotation (around COM) */
        vehicle->translate(state.linearVelocity*parameters.physicsTimeDelta)
            ->rotate(Rad(state.angularSpeed*parameters.physicsTimeDelta), SceneGraph::TransformationType::Local)
            ->normalizeRotation();

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
    /* Compute tangent and normal vectors */
    const Vector2 engineDirectionLeft(engineLeft->absoluteTransformation().rotation());
    const Vector2 engineDirectionRight(engineRight->absoluteTransformation().rotation());

    /* Reset forces */
    forces.totalLeftArm = forces.totalRightArm =
        forces.frictionLeftArm = forces.frictionRightArm =
            forces.spring[0] = forces.spring[1] = forces.spring[2] = {};

    /* Add engine forces */
    forces.engineLeftArm = engineDirectionLeft*state.currentPowerLeftArm;
    forces.engineRightArm = engineDirectionRight*state.currentPowerRightArm;
    forces.totalLeftArm += forces.engineLeftArm;
    forces.totalRightArm += forces.engineRightArm;

    /* Apply forces */
    applyForce(vehicle->absoluteTransformation().translation(), forces.weight);
    applyForce(engineLeft->absoluteTransformation().translation(), forces.totalLeftArm);
    applyForce(engineLeft->absoluteTransformation().translation(), forces.totalRightArm);

    /* Apply spring forces for all penetrations */
    collisionShapes.setClean();
    for(std::size_t i = 0; i != 3; ++i) {
        const auto& p = shapes.vehicle->transformedShape().get<Shapes::Point2D>(i);
        if(p % shapes.tubeMin->transformedShape()) continue;

        /* Spring force is proportional to penetration depth */
        forces.spring[i] = p.position()*(parameters.springConstant*
            (shapes.tubeMin->shape().radius()*p.position().lengthInverted() - 1.0f));
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
