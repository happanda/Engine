#ifndef INCLUDE_SHAPE
#define INCLUDE_SHAPE

#include "Math\Geometry.h"

enum shape_type
{
   sh_rectangle,
   sh_circle,
   sh_surface
};

struct bbox
{
   double left;
   double right;
   double top;
   double bottom;
   bbox ();
   bbox (double left, double right, double top, double bottom);
};

struct shape
{
public:
   Vector2 point;// center of mass
   double alpha;// rotation angle
   shape_type type;
   void rotate(double angle);
   shape();
   shape(shape_type sh, double point_x, double point_y, double alpha);
   virtual ~shape();
   virtual bbox bounding_box() const = 0;
};

struct rectangle : public shape
{
private:
   double half_diag;
public:
   double h, w;// sides lengths
   rectangle();
   rectangle(double point_x, double point_y, double alpha, double height, double width);
   ~rectangle();
   void get_points(Vector2& p1, Vector2& p2, Vector2& p3, Vector2& p4, double indent) const;
   bbox bounding_box() const;
};

struct circle : public shape
{
public:
   circle();
   circle(double point_x, double point_y, double alpha, double radius);
   ~circle();
   double radius;
   bbox bounding_box() const;
};

struct surface : public shape
{
};

#endif