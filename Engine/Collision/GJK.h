#ifndef INCLUDE_GJK
#define INCLUDE_GJK

#include <deque>
#include "Math\Geometry.h"
#include "Body\Shape.h"
#include "Collision\Collision.h"

const double GJK_INDENT_EPSILON = 0.01;
const double GJK_INDENT_EPSILON_CHECK = GJK_INDENT_EPSILON * 2;

enum simplex_min_norm
{
   SIMPLEX_A_POINT = 0,
   SIMPLEX_B_POINT = 1,
   SIMPLEX_C_POINT = 2,
   SIMPLEX_AB_EDGE = 3,
   SIMPLEX_BC_EDGE = 4,
   SIMPLEX_CA_EDGE = 5
};

class Simplex
{
public:
   std::deque<Vector2> A;
   std::deque<Vector2> B;
   std::deque<Vector2> P;
   simplex_min_norm feature;
   void push(Vector2 a, Vector2 b, Vector2 p);
   void insert(Vector2 a, Vector2 b, Vector2 p, size_t place);
   void clear();
   void pop();
   void pop(Vector2& a, Vector2& b, Vector2& p);
   void leave_1();
   void leave_1_2();
   void leave_1_3();
   size_t size() { return P.size(); }
   double norm();
};

Vector2 gjk_support(Vector2 direction, shape& sh);

bool gjk_check_collision(shape& shapeA, shape& shapeB,
                        Vector2 (*support)(Vector2 direction, shape& sh),
                        Collision& collision);

bool gjk_process_simplex(Simplex& simplex, Vector2& direction);

void epa_get_features(shape& shapeA, shape& shapeB,
                      Vector2 (*support)(Vector2 direction, shape& sh),
                      Simplex& simplex, Collision& collision);

#endif