#ifndef INCLUDE_COLLISION
#define INCLUDE_COLLISION

#include <vector>
#include "Math\MathRoutines.h"
#include "Body\Body.h"
#include "Body\Shape.h"

class Collision
{
public:
   Collision(Body* bodyA, Body* bodyB);
   Collision(const Collision& other);
   const Collision& operator=(const Collision& other);
   Body* BodyA;
   Body* BodyB;
   // points of contact (one point or an edge)
   Vector2 pointsA[2];
   Vector2 pointsB[2];
   // from second body to first body
   Vector2 normal;
   // set contact points of A body
   void setApoints(Vector2* points, size_t size);
   // set contact points of B body
   void setBpoints(Vector2* points, size_t size);
   size_t sizeA() const;
   size_t sizeB() const;
private:
   static const size_t MAX_CONTACT_POINTS = 2;
   // indeces in pointsA and pointsB
   size_t m_sizeA;
   size_t m_sizeB;
};

void gjk_collide(std::vector<Body>& bodies, std::vector<Collision>& collisions);

bool bbox_check_collision(Body* bodyA, Body* bodyB);

bool check_point_inside(const Vector2 point, const Body* body, Vector2& localCoord);

#endif