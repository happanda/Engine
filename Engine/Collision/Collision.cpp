
#include "Collision.h"
#include "Collision\GJK.h"
#include "SAT.h"

Collision::Collision(Body* bodyA, Body* bodyB): BodyA(bodyA), BodyB(bodyB)
{
}
Collision::Collision(const Collision& other): BodyA(other.BodyA), BodyB(other.BodyB)
{
   for (size_t i = 0; i < other.m_sizeA; ++i)
      pointsA[i] = other.pointsA[i];
   m_sizeA = other.m_sizeA;
   for (size_t i = 0; i < other.m_sizeB; ++i)
      pointsB[i] = other.pointsB[i];
   m_sizeB = other.m_sizeB;
   normal = other.normal;
}
const Collision& Collision::operator=(const Collision& other)
{
   BodyA = other.BodyA;
   BodyB = other.BodyB;
   for (size_t i = 0; i < other.m_sizeA; ++i)
      pointsA[i] = other.pointsA[i];
   m_sizeA = other.m_sizeA;
   for (size_t i = 0; i < other.m_sizeB; ++i)
      pointsB[i] = other.pointsB[i];
   m_sizeB = other.m_sizeB;
   normal = other.normal;
   return *this;
}
void Collision::setApoints(Vector2* points, size_t size)
{
   size_t i = 0;
   for (; i < MAX_CONTACT_POINTS && i < size; ++i)
      pointsA[i] = points[i];
   m_sizeA = i;
}
void Collision::setBpoints(Vector2* points, size_t size)
{
   size_t i = 0;
   for (; i < MAX_CONTACT_POINTS && i < size; ++i)
      pointsB[i] = points[i];
   m_sizeB = i;
}
size_t Collision::sizeA() const
{
   return m_sizeA;
}
size_t Collision::sizeB() const
{
   return m_sizeB;
}

void gjk_collide(std::vector<Body>& bodies, std::vector<Collision>& collisions)
{
    collisions.clear();
    for (std::vector<Body>::iterator it = bodies.begin(); it != bodies.end(); it++)
    {
        for (std::vector<Body>::iterator jt = it; jt != bodies.end(); jt++)
        {
            if (it != jt && bbox_check_collision(&(*it), &(*jt)))
            {
                Collision coll(&(*it), &(*jt));
                if (gjk_check_collision(*(it->form), *(jt->form), &gjk_support, coll))
                {
                    collisions.push_back(coll);
                }
            }
        }
    }
}

bool bbox_check_collision(Body* bodyA, Body* bodyB)
{
    bbox bboxA = bodyA->form->bounding_box();
    bbox bboxB = bodyB->form->bounding_box();
    if (bboxA.right < bboxB.left || bboxB.right < bboxA.left
        || bboxA.top < bboxB.bottom || bboxB.top < bboxA.bottom)
        return false;
    return true;
}

bool check_point_inside(const Vector2 point, const Body* body, Vector2& localCoord)
{
    const shape* sh = body->form;
    if (sh->type == sh_rectangle)
    {
        const rectangle* rect = static_cast<const rectangle*>(sh);
        localCoord = point - rect->point;
        Vector2 p[4];
        rect->get_points(p[0], p[1], p[2], p[3], 0);
        for (size_t i = 0; i < 3; i++)
            if ((point - p[i]) * (p[i + 1] - p[i]) < 0)
                return false;
        if ((point - p[3]) * (p[0] - p[3]) < 0)
            return false;
        return true;
    }
    if (sh->type == sh_circle)
    {
        const circle* circ = static_cast<const circle*>(sh);
        localCoord = point - circ->point;
        if (localCoord.norm2() > circ->radius)
            return false;
        return true;
    }
    return false;
}