#include "gjk.h"
#include <assert.h>
#include <float.h>
#include "geometry.h"
#include "math_routines.h"
#include "draw.h"
#include <conio.h>

bool gjk_check_collision(shape& shapeA, shape& shapeB,
                        Vector2 (*support)(Vector2 direction, shape& sh),
                        Collision& collision)
{
   Simplex simplex;
   Vector2 direction = shapeB.point - shapeA.point;
   direction = direction.perpendicular();
   direction.normalize2();
   //Vector2 indent = direction * GJK_INDENT_EPSILON;
   Vector2 indent(0, 0);
   Vector2 suppA = support(direction, shapeA) - indent;
   Vector2 suppB = support(-direction, shapeB) - indent;
   Vector2 s = suppA - suppB;

   simplex.push(suppA, suppB, s);
   direction = -s;
   int maxIter = 100,
      numIter = 0;
   simplex.feature = SIMPLEX_A_POINT;
   while(numIter < maxIter)
   {
      numIter++;
      //indent = direction * GJK_INDENT_EPSILON;
      suppA = support(direction, shapeA) - indent;
      suppB = support(-direction, shapeB) - indent;
      s = suppA - suppB;
      if (s * direction < 0)
      {
         /*double snorm = simplex.norm();
         if (snorm < GJK_INDENT_EPSILON_CHECK)
         {
            gjk_get_features(simplex, collision);
            return true;
         }*/
         return false;
      }
      simplex.push(suppA, suppB, s);
      if (gjk_process_simplex(simplex, direction))
      {
         //gjk_get_features(simplex, collision);
         epa_get_features(shapeA, shapeB, support, simplex, collision);
         return true;
      }
   }
   return false;
}

bool gjk_process_simplex(Simplex& simplex, Vector2& direction)
{
   assert(simplex.size() > 1);

   bool intersect = false;
   Vector2 A = simplex.P[0],
      B = simplex.P[1];
   Vector2 AB = B - A,
      AO = Vector2::ORIGIN - A;
   switch (simplex.size())
   {
   case 2:
      {
         if (same_direction(AB, AO))
         {
            if (isOn(Vector2::ORIGIN, A, B))
               intersect = true;
            direction = perpendicular(AB, -AO);
            simplex.feature = SIMPLEX_AB_EDGE;
         }
         else
         {
            simplex.feature = SIMPLEX_A_POINT;
            direction = AO;
         }
         break;
      }
   case 3:
      {
         Vector2 C = simplex.P[2];
         Vector2 AC = C - A;

         Vector2 ofAC = perpendicular(AC, AB);
         Vector2 ofAB = perpendicular(AB, AC);

         if (same_direction(ofAC, AO))
         {
            if (same_direction(AC, AO))
            {
               //printf("AC || AO\n");
               simplex.leave_1_3();
               direction = ofAC;
               simplex.feature = SIMPLEX_CA_EDGE;
            }
            else
            {
               /* duplicate code */
               simplex.leave_1();
               direction = AO;
               simplex.feature = SIMPLEX_A_POINT;
            }
         }
         else
         {
            if (same_direction(ofAB, AO))
            {
               if (same_direction(AB, AO))
               {
                  //printf("AB || AO\n");
                  simplex.leave_1_2();
                  direction = ofAB;
                  simplex.feature = SIMPLEX_AB_EDGE;
               }
               else
               {
                  //printf("AB <> AO");
                  /* duplicate code */
                  simplex.leave_1();
                  direction = AO;
                  simplex.feature = SIMPLEX_A_POINT;
               }
            }
            else
            {
               intersect = true;
            }
         }
         break;
      }
   }
   return intersect;
}

void gjk_get_edge_features(const Simplex& simplex, Collision& collision,
                   size_t edgeP1, size_t edgeP2)
{
   // these conditions
   // (simplex.A[0] - simplex.A[1]).norm2sq() < DBL_EPSILON
   // and
   // (simplex.B[0] - simplex.B[1]).norm2sq() < DBL_EPSILON
   // can't be true simultaneously
   if ((simplex.A[edgeP1] - simplex.A[edgeP2]).norm2sq() < DBL_EPSILON)
   {
      collision.one.p = simplex.A[edgeP1];
      collision.two.p = closest_point(collision.one.p,
         Segment(simplex.B[edgeP1], simplex.B[edgeP2]));
      collision.normal = perpendicular(simplex.B[edgeP2] - simplex.B[edgeP1],
         collision.body_two->form->point);
   }
   else if ((simplex.B[edgeP1] - simplex.B[edgeP2]).norm2sq() < DBL_EPSILON)
   {
      Body* body_tmp = collision.body_one;
      collision.body_one = collision.body_two;
      collision.body_two = body_tmp;

      collision.one.p = simplex.B[edgeP1];
      collision.two.p = closest_point(collision.one.p,
         Segment(simplex.A[edgeP1], simplex.A[edgeP2]));
      collision.normal = perpendicular(simplex.A[edgeP2] - simplex.A[edgeP1],
         collision.one.p - collision.two.p);
   }
   else
   {
      collision.edge_edge = true;
      collision.one.segm = Segment(simplex.A[edgeP1], simplex.A[edgeP2]);
      collision.two.segm = Segment(simplex.B[edgeP1], simplex.B[edgeP2]);
      collision.normal = perpendicular(simplex.B[edgeP2] - simplex.B[edgeP1],
         collision.body_two->form->point - collision.body_one->form->point);
   }
}

void epa_get_features(shape& shapeA, shape& shapeB,
                      Vector2 (*support)(Vector2 direction, shape& sh),
                      Simplex& simplex, Collision& collision)
{
   int maxIter = 100,
      numIter = 0;
   while(numIter < maxIter)
   {
      numIter++;
      size_t simpl_size = simplex.size();
      double min_dist = DBL_MAX;
      int min_ind = 0;
      for (size_t s_num = 0; s_num < simpl_size; s_num++)
      {
         size_t s_num_next = (s_num + 1) % simpl_size;
         Segment edge(simplex.P[s_num], simplex.P[s_num_next]);
         double dist = distance(Vector2::ORIGIN, edge);
         if (dist < min_dist)
         {
            min_dist = dist;
            min_ind = s_num;
         }
      }
      Vector2 min_edge_vect = simplex.P[(min_ind + 1) % simpl_size] -
         simplex.P[min_ind];
      Vector2 direction = perpendicular(min_edge_vect, -simplex.P[min_ind]);
      direction.normalize2();

      Vector2 suppA = support(direction, shapeA);
      Vector2 suppB = support(-direction, shapeB);
      Vector2 s = suppA - suppB;
      // the edge is on the Minkowsky difference, and it's the closest
      if (abs(s * direction - min_dist) < DBL_EPSILON * 1000)
      {
         gjk_get_edge_features(simplex, collision, min_ind, (min_ind + 1) % simpl_size);
         collision.normal.normalize2();
         return;
      }
      simplex.insert(suppA, suppB, s, (min_ind + 1) % simpl_size);
   }
}

void gjk_get_features(const Simplex& simplex,
                       Collision& collision)
{
   collision.edge_edge = false;
   switch(simplex.feature)
   {
      case SIMPLEX_A_POINT:
         //printf("SIMPLEX_A_POINT\n");
         collision.one.p = simplex.A[0];
         collision.two.p = simplex.B[0];
         collision.normal = collision.one.p - collision.two.p;
         break;
      case SIMPLEX_B_POINT:
         //printf("SIMPLEX_B_POINT\n");
         collision.one.p = simplex.A[1];
         collision.two.p = simplex.B[1];
         collision.normal = collision.one.p - collision.two.p;
         break;
      case SIMPLEX_C_POINT:
         //printf("SIMPLEX_C_POINT\n");
         collision.one.p = simplex.A[2];
         collision.two.p = simplex.B[2];
         collision.normal = collision.one.p - collision.two.p;
         break;
      case SIMPLEX_AB_EDGE:
         //printf("SIMPLEX_AB_POINT\n");
         gjk_get_edge_features(simplex, collision, 0, 1);
         break;
      case SIMPLEX_BC_EDGE:
         //printf("SIMPLEX_BC_POINT\n");
         gjk_get_edge_features(simplex, collision, 1, 2);
         break;
      case SIMPLEX_CA_EDGE:
         //printf("SIMPLEX_CA_POINT\n");
         gjk_get_edge_features(simplex, collision, 2, 0);
         break;
   }
   collision.normal.normalize2();
}

void Simplex::push(Vector2 a, Vector2 b, Vector2 p)
{
   A.push_front(a);
   B.push_front(b);
   P.push_front(p);
}
void Simplex::insert(Vector2 a, Vector2 b, Vector2 p, size_t place)
{
   A.insert(A.begin() + place, a);
   B.insert(B.begin() + place, b);
   P.insert(P.begin() + place, p);
}
void Simplex::clear()
{
   A.clear();
   B.clear();
   P.clear();
}
void Simplex::pop()
{
   A.pop_front();
   B.pop_front();
   P.pop_front();
}
void Simplex::pop(Vector2& a, Vector2& b, Vector2& p)
{
   a = A.front();
   b = B.front();
   p = P.front();
   pop();
}
void Simplex::leave_1()
{
   Vector2 a0, b0, p0;
   pop(a0, b0, p0);
   clear();
   push(a0, b0, p0);
}
void Simplex::leave_1_2()
{
   Vector2 a0, b0, p0, a1, b1, p1;
   pop(a0, b0, p0);
   pop(a1, b1, p1);
   clear();
   push(a1, b1, p1);
   push(a0, b0, p0);
}
void Simplex::leave_1_3()
{
   Vector2 a0, b0, p0, a2, b2, p2;
   pop(a0, b0, p0);
   pop();
   pop(a2, b2, p2);
   clear();
   push(a2, b2, p2);
   push(a0, b0, p0);
}
double Simplex::norm()
{
   assert(size() > 0);

   double norms[6];
   int count = 1;

   Vector2 A = P[0];
   norms[0] = A.norm2();
   if (size() > 1)
   {
      count = 3;
      Vector2 B = P[1];
      Segment AB(A, B);
      norms[1] = B.norm2();
      norms[2] = distance(Vector2::ORIGIN, AB);
      if (size() > 2)
      {
         count = 6;
         Vector2 C = P[2];
         Segment BC(B, C),
            CA(C, A);
         norms[3] = C.norm2();
         norms[4] = distance(Vector2::ORIGIN, BC);
         norms[5] = distance(Vector2::ORIGIN, CA);
      }
   }
   int ind = 0;
   for (int i = 0; i < count; i++)
      if (norms[i] < norms[ind])
         ind = i;
   switch (ind)
   {
   case 0:
      feature = SIMPLEX_A_POINT;
      break;
   case 1:
      feature = SIMPLEX_B_POINT;
      break;
   case 2:
      feature = SIMPLEX_AB_EDGE;
      break;
   case 3:
      feature = SIMPLEX_C_POINT;
      break;
   case 4:
      feature = SIMPLEX_BC_EDGE;
      break;
   case 5:
      feature = SIMPLEX_CA_EDGE;
      break;
   }
   return norms[ind];
}

Vector2 gjk_support(Vector2 direction, shape& sh)
{
   Vector2 point;
   switch (sh.type)
   {
   case sh_circle:
      {
         circle* circ = static_cast<circle*>(&sh);
         double norm = direction.norm2();
         point = direction * (circ->radius / norm) + circ->point;
         break;
      }
   case sh_rectangle:
      {
         rectangle* rect = static_cast<rectangle*>(&sh);
         Vector2 p1, p2, p3, p4;
         get_points(*rect, p1, p2, p3, p4, 0);
         double max = direction * p1;
         point = p1;
         double next = direction * p2;
         if (next > max)
         {
            max = next;
            point = p2;
         }
         next = direction * p3;
         if (next > max)
         {
            max = next;
            point = p3;
         }
         next = direction * p4;
         if (next > max)
         {
            max = next;
            point = p4;
         }
         break;
      }
   }
   return point;
}