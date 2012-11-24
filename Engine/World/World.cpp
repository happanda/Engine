
#include "world.h"
#include <stdio.h>
#include "Collision\GJK.h"
#include "Math\MathRoutines.h"
#include "Graphics\Draw.h"
#include "Constraints\ContactConstraint.h"
#include "Constraints\FrictionConstraint.h"
#include "Constraints\DoFConstraint.h"

World::World()
{
    init();
}

void World::init()
{
    bodies.clear();
    collisions.clear();
    constraints.clear();
    // arbitrary numbers
    bodies.reserve(100);
    collisions.reserve(1000);
    constraints.reserve(100);
    vars.timeStep = 0.016;
    vars.iTimeStep = 1 / vars.timeStep;
    vars.UNMOVABLE_MASS = 5000;
    vars.RESTITUTION = 0.5;
    vars.FRICTION = 0.4;
    vars.GRAVITATION = Vector2(0, -9.8);
    force_id = 0;
}

void World::update(double deltaT)
{
    force_ext[0] = vars.GRAVITATION.v1;
    force_ext[1] = vars.GRAVITATION.v2;
    force_ext[2] = 0;

    force_ext[3] = vars.GRAVITATION.v1;
    force_ext[4] = vars.GRAVITATION.v2;
    force_ext[5] = 0;

    apply_forces(deltaT);
    gjk_collide(bodies, collisions);
    resolve_collision(deltaT);
    resolve_constraints(deltaT);
    double energy = 0;
    for (std::vector<Body>::iterator it = bodies.begin(); it != bodies.end(); ++it)
    {
        it->form->rotate(it->angle_vel * deltaT);
        it->form->point = it->form->point + it->velocity * deltaT;
        energy += (it->mass * it->velocity.norm2sq()
            + it->inert * it->angle_vel * it->angle_vel) / 2;
    }
    // printf("Total kinetic energy: %.2f\n", energy);
}

void World::resolve_collision(double deltaT)
{
    // adjusted parameters affecting interpenetration
    const double min_impulse = 0.0000001;
    const double bias_factor = 0.3;
    const double delta_slop = 0.002;
    for (std::vector<Collision>::iterator it = collisions.begin(); it != collisions.end(); ++it)
    {
        if (it->BodyA->mass < vars.UNMOVABLE_MASS
            || it->BodyB->mass < vars.UNMOVABLE_MASS)
        {
            for (size_t pind = 0; pind < it->sizeA(); pind++)
            {
                ContactConstraint cc(&(*it), pind, &vars);
                FrictionConstraint fc(&(*it), pind, &vars);
                cc.SetForceExt(force_ext);
                fc.SetForceExt(force_ext);
                for (size_t num_iter = 0; num_iter < cc.NumIter() && !cc.Enough(); num_iter++)
                {
                    if (num_iter > 0 && cc.Impulse.norm2sq() <= min_impulse && fc.Impulse.norm2sq() <= min_impulse)
                        break;
                    cc.DeltaImpulse();
                    fc.AppliedNormalImpulse = cc.Impulse.norm2();
                    fc.DeltaImpulse();
                    cc.ApplyImpulse();
                    fc.ApplyImpulse();
                }
            }
        }
    }
}

void World::resolve_constraints(double deltaT)
{
    const double min_impulse = 0.0000001;
    for (std::vector<Constraint*>::iterator it = constraints.begin(); it != constraints.end(); ++it)
    {
        Constraint* cc = *it;
        if (cc->bodyA->mass < vars.UNMOVABLE_MASS)
        {
            cc->SetForceExt(force_ext);
            for (size_t num_iter = 0; num_iter < cc->NumIter() && !cc->Enough(); num_iter++)
            {
                cc->DeltaImpulse();
                cc->ApplyImpulse();
            }
        }
    }
}

void World::resolve_collision_deprecated(double deltaT)
{
    const double min_impulse = 0.0000001;
    const double bias_factor = 0.3;
    const double delta_slop = 0.002;
    for (std::vector<Collision>::iterator it = collisions.begin(); it != collisions.end(); ++it)
    {
        Vector2 tang = it->normal.perpendicular();

        int maxIter = 100;
        int numIter = 0;
        bool enough = false;
        double sum_impulse_n = 0,// normal direction total impulse
            sum_impulse_t = 0;// tangential direction total impulse

        // iteratively apply impulses
        while (!enough && numIter < maxIter)
        {
            numIter++;
            // iterate through collision points
            for (size_t pind = 0; pind < it->sizeA(); pind++)
            {
                Vector2 vel_one = it->BodyA->point_velocity(it->pointsA[pind]);
                Vector2 vel_two = it->BodyB->point_velocity(it->pointsB[pind]);
                Vector2 vel_rel = vel_one - vel_two;
                double vel_rel_n = vel_rel * it->normal;
                if (vel_rel_n < 0)
                {
                    double vel_rel_t = vel_rel * tang;

                    double m1 = it->BodyA->mass;
                    double m2 = it->BodyB->mass;
                    double iinrt1 = it->BodyA->iInert;
                    double iinrt2 = it->BodyB->iInert;

                    // relative positions of contact points
                    Vector2 ra_2d = it->pointsA[pind] - it->BodyA->form->point;
                    Vector2 rb_2d = it->pointsB[pind] - it->BodyB->form->point;
                    Vector2 a_add_n(0, 0), b_add_n(0, 0),
                        a_add_t(0, 0), b_add_t(0, 0);
                    double mm = 0;

                    if (m1 < vars.UNMOVABLE_MASS)
                    {
                        mm += it->BodyA->iMass;
                        a_add_n = cross_cross(ra_2d, it->normal) * iinrt1;
                        a_add_t = cross_cross(ra_2d, tang) * iinrt1;
                    }
                    if (m2 < vars.UNMOVABLE_MASS)
                    {
                        mm += it->BodyB->iMass;
                        b_add_n = cross_cross(rb_2d, it->normal) * iinrt2;
                        b_add_t = cross_cross(rb_2d, tang) * iinrt2;
                    }

                    double kn = mm + it->normal * (a_add_n + b_add_n);
                    double kt = mm + tang * (a_add_t + b_add_t);
                    double delta = (it->pointsA[pind] - it->pointsB[pind]).norm2();
                    double v_bias = 0;
                    // penetration correction impulse
                    if (delta > delta_slop)
                        v_bias = bias_factor * (delta - delta_slop) / deltaT;
                    double Pn = (-(1 + vars.RESTITUTION) * vel_rel_n + v_bias) / kn;
                    double Pt = -vars.FRICTION * vel_rel_t / kt;

                    if (Pn > 0 && Pt < -vars.FRICTION * Pn)
                        Pt = -vars.FRICTION * Pn;
                    else if (Pn > 0 && Pt > vars.FRICTION * Pn)
                        Pt = vars.FRICTION * Pn;

                    if (abs(Pn) < min_impulse && abs(Pt) < min_impulse)
                        enough = true;
                    // correcting impulses
                    if (sum_impulse_n + Pn < 0)
                        Pn = -sum_impulse_n;
                    sum_impulse_n += Pn;
                    if (sum_impulse_t + Pt < 0)
                        Pt = -sum_impulse_t;
                    sum_impulse_t += Pt;

                    // total impulse
                    Vector2 P = it->normal * Pn + tang * Pt;

                    Vector2 deltaV1(0, 0);
                    Vector2 deltaV2(0, 0);

                    double deltaW1 = 0;
                    double deltaW2 = 0;

                    if (m1 < vars.UNMOVABLE_MASS)
                    {
                        deltaV1 = P * it->BodyA->iMass;
                        deltaW1 = iinrt1 * (ra_2d.v1 * P.v2 - ra_2d.v2 * P.v1);
                    }
                    if (m2 < vars.UNMOVABLE_MASS)
                    {
                        deltaV2 = P * (-it->BodyB->iMass);
                        deltaW2 = -iinrt2 * (rb_2d.v1 * P.v2 - rb_2d.v2 * P.v1);
                    }
                    it->BodyA->velocity = it->BodyA->velocity + deltaV1;
                    it->BodyA->angle_vel += deltaW1;
                    it->BodyB->velocity = it->BodyB->velocity + deltaV2;
                    it->BodyB->angle_vel += deltaW2;
                }
                else
                    enough = true;
            }// for points
        }// while iterations
    }// for collisions
}

void World::apply_forces(double deltaT)
{
    for (std::vector<Body>::iterator it = bodies.begin(); it != bodies.end(); ++it)
    {
        if (it->mass < vars.UNMOVABLE_MASS)
            it->velocity = it->velocity + vars.GRAVITATION * deltaT;
    }
    for (std::vector<Force>::iterator it = forces.begin(); it != forces.end(); ++it)
    {
        if (it->Body->mass < vars.UNMOVABLE_MASS)
            it->Apply();
    }
}

void World::addBody(Body body)
{
    bodies.push_back(body);
}

void World::addRope(Rope rope)
{
    ropes.push_back(rope);
}

void World::addConstraint(Constraint* constraint)
{
    constraints.push_back(constraint);
}

void World::addForce(Force force)
{
    force.TimeStep = &(vars.timeStep);
    force.id = force_id++;
    forces.push_back(force);
}

void World::removeForce(int force_id)
{
    for (size_t i = 0; i < forces.size(); i++)
    {
        if (forces[i].id == force_id)
        {
            forces.erase(forces.begin() + i);
            return;
        }
    }
}