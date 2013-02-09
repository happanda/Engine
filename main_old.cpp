void stack_init();

void init_bodies1();
void init_bodies2();
void init_bodies3();
void init_angry_circles();
void init_many_rectangles();
void init_many_circles();

    if (key == '1') { glut_init(); init_bodies1(); tw_init(); draw_tw = true; }
    //if (key == '0') { test_gjk_init(); draw_tw = false; }
    if (key == '2') { glut_init(); init_bodies2(); tw_init(); draw_tw = true; }
    if (key == '3') { glut_init(); init_bodies3(); tw_init(); draw_tw = true; }
    if (key == '4') { glut_init(); init_many_rectangles(); tw_init(); draw_tw = true; }
    if (key == '5') { glut_init(); init_many_circles(); tw_init(); draw_tw = true; }
    if (key == '6') { glut_init(); init_angry_circles(); tw_init(); draw_tw = true; }


    if (value == 1) { glut_init(); init_bodies1(); tw_init(); draw_tw = true; }
    if (value == 2) { test_gjk_init(); draw_tw = false; }
    if (value == 3) { glut_init(); stack_init(); tw_init(); draw_tw = true; }
    if (value == 4) { glut_init(); init_many_rectangles(); tw_init(); draw_tw = true; }
    if (value == 5) { glut_init(); init_many_circles(); tw_init(); draw_tw = true; }


void init_bodies1()
{
    world_vars wvars = world.vars;
    wvars.GRAVITATION.v2 = -20;
    wvars.RESTITUTION = 0.9;
    wvars.FRICTION = 0.4;
    //wvars.RESTITUTION = 1;
    world.init();
    world.vars = wvars;
    double angle_vel = 0;

    double vel = 4;
    world.addBody(new Body(new circle(-7, 60, 0, 2), 6, 0, 0, 0));
    world.addBody(new Body(new circle(7, -10.8, 0, 1), 2, 0, 0, 0));
    world.addBody(new Body(new circle(7, -8.8, 0, 1), 1, 0, 0, 0));
    world.addBody(new Body(new circle(7, -6.8, 0, 1), 1, 0, 0, 0));
    world.addBody(new Body(new circle(7, -4.8, 0, 1), 1, 0, 0, 0));
    world.addBody(new Body(new circle(7, -2.8, 0, 1), 1, 0, 0, 0));
    world.addBody(new Body(new circle(7, -0.8, 0, 1), 1, 0, 0, 0));

    world.addBody(new Body(new rectangle(0, -10, -0.28666, 16, 0.5), 8, 0, 0, 0));

    /*world.addBody(new Body(new circle(-5, 5, 0, 1), 1, -vel, 0, 0));
    world.addBody(new Body(new circle(5, 5, 0, 1), 1, vel, 0, 0));
    world.addBody(new Body(new circle(-5, 15, 0, 1), 1, -vel, 0, 0));
    world.addBody(new Body(new circle(5, 15, 0, 1), 1, vel, 0, 0));*/

    // bounds
    double bigmass = wvars.UNMOVABLE_MASS;
    /*world.addBody(new Body(new rectangle(0, 2, 0, 2, 29), bigmass, 0, 0, 0));
    world.addBody(new Body(new rectangle(20, 2, 0, 2, 29), bigmass, 0, 0, 0));
    world.addBody(new Body(new rectangle(-20, 2, 0, 2, 29), bigmass, 0, 0, 0));
    world.addBody(new Body(new rectangle(0, 18, 0, 2000, 3), bigmass, 0, 0, 0));*/
    world.addBody(new Body(new rectangle(0, -14, 0, 2000, 3), bigmass, 0, 0, 0));
    // bounds

    // some simple axis constraints
    int ind = 0;
    world.addConstraint(new DoFConstraint(world.bodies[ind++], X_AXIS, &(world.vars)));
    world.addConstraint(new DoFConstraint(world.bodies[ind++], X_AXIS, &(world.vars)));
    world.addConstraint(new DoFConstraint(world.bodies[ind++], X_AXIS, &(world.vars)));
    world.addConstraint(new DoFConstraint(world.bodies[ind++], X_AXIS, &(world.vars)));
    world.addConstraint(new DoFConstraint(world.bodies[ind++], X_AXIS, &(world.vars)));
    world.addConstraint(new DoFConstraint(world.bodies[ind++], X_AXIS, &(world.vars)));
    world.addConstraint(new DoFConstraint(world.bodies[ind++], X_AXIS, &(world.vars)));

    world.addConstraint(new DoFConstraint(world.bodies[ind++], XY_AXIS, &(world.vars)));
}

void init_bodies2()
{
    world_vars wvars = world.vars;
    wvars.GRAVITATION.v2 = -9.8;
    wvars.RESTITUTION = 0.5;
    wvars.FRICTION = 0.4;
    world.init();
    world.vars = wvars;
    double angle_vel = 0;

    double vel = 4;
    world.addBody(new Body(new rectangle(0, 11, 0, 7.8, 0.5), 50, 0, 0, 0));
    world.addBody(new Body(new rectangle(0, 3, 0, 7.8, 0.5), 50, 0, 0, 0));
    world.addBody(new Body(new rectangle(0, -5, 0, 7.8, 0.5), 50, 0, 0, 0));
    world.addBody(new Body(new rectangle(0, -13, 0, 7.8, 0.5), 50, 0, 0, 0));

    int minx = -3, maxx = 3;
    double y = 15;
    int s = 80;
    double sp = (double)RAND_MAX / 12;
    for (int i = 0; i < s; i++)
    {
        double rx = rand() % (maxx - minx) + minx;
        world.addBody(new Body(new circle(rx, y, (double)rand(), 0.5),
            1, 0, 0, 0));
        y += 3;
    }

    /*world.addBody(new Body(new circle(-5, 5, 0, 1), 1, -vel, 0, 0));
    world.addBody(new Body(new circle(5, 5, 0, 1), 1, vel, 0, 0));
    world.addBody(new Body(new circle(-5, 15, 0, 1), 1, -vel, 0, 0));
    world.addBody(new Body(new circle(5, 15, 0, 1), 1, vel, 0, 0));*/

    // bounds
    double bigmass = wvars.UNMOVABLE_MASS;
    world.addBody(new Body(new rectangle(-5, 0, 0, 2, 200), bigmass, 0, 0, 0));
    world.addBody(new Body(new rectangle(5, 0, 0, 2, 200), bigmass, 0, 0, 0));
    //world.addBody(new Body(new rectangle(0, 18, 0, 12, 3), bigmass, 0, 0, 0));
    world.addBody(new Body(new rectangle(0, -101, 0, 12, 3), bigmass, 0, 0, 0));
    // bounds

    // some simple axis constraints
    world.addConstraint(new DoFConstraint(world.bodies[0], XY_AXIS, &(world.vars)));
    world.addConstraint(new DoFConstraint(world.bodies[1], XY_AXIS, &(world.vars)));
    world.addConstraint(new DoFConstraint(world.bodies[2], XY_AXIS, &(world.vars)));
    world.addConstraint(new DoFConstraint(world.bodies[3], XY_AXIS, &(world.vars)));
    //world.addConstraint(new DoFConstraint(&world.bodies[4], XY_AXIS, &(world.vars)));
}

void init_bodies3()
{
    world_vars wvars = world.vars;
    wvars.GRAVITATION.v2 = -0;
    wvars.RESTITUTION = 0.5;
    wvars.FRICTION = 0.4;
    world.init();
    world.vars = wvars;
    
    double bigmass = wvars.UNMOVABLE_MASS;
    world.addBody(new Body(new rectangle(-40, 0, 0, 20, 25), bigmass, 0, 0, 0));
    world.addBody(new Body(new rectangle(40, 0, 0, 20, 25), bigmass, 0, 0, 0));
    world.addBody(new Body(new rectangle(0, 14, 0, 200, 3), bigmass, 0, 0, 0));
    world.addBody(new Body(new rectangle(0, -14, 0, 200, 3), bigmass, 0, 0, 0));


    Destructable* destr = new Destructable(Vector2::ORIGIN, 2, 2, 100, &world.vars);
    world.addDestructable(destr);

    /*world.addBody(new Body(new rectangle(-12, 0, 0, 2, 4), 1, 0, 0, 0));
    world.addBody(new Body(new rectangle(-8, 0, 0, 2, 4), 1, 0, 0, 0));
    world.addBody(new Body(new rectangle(-4, 0, 0, 2, 4), 1, 0, 0, 0));
    world.addBody(new Body(new rectangle(0, 0, 0, 2, 4), 1, 0, 0, 0));
    world.addConstraint(new FixedConstraint(world.bodies[5], Vector2(-1, 2), world.bodies[4],
        Vector2(1, 2), &(world.vars)));
    world.addConstraint(new FixedConstraint(world.bodies[5], Vector2(-1, -2), world.bodies[4],
        Vector2(1, -2), &(world.vars)));*/
    /*world.addConstraint(new FixedConstraint(world.bodies[5], Vector2::ORIGIN, world.bodies[6],
        Vector2::ORIGIN, &(world.vars)));
    world.addConstraint(new FixedConstraint(world.bodies[6], Vector2::ORIGIN, world.bodies[7],
        Vector2::ORIGIN, &(world.vars)));*/
    
    /*Chain* chain = new Chain(Vector2(8, 2), 15, 15, &world.vars);
    world.addChain(chain);
    world.addConstraint(new DoFConstraint(chain->points.front(), XY_AXIS, &(world.vars)));
    world.addConstraint(new DoFConstraint(chain->points.back(), XY_AXIS, &(world.vars)));*/

    /*Rope* rope = new Rope(Vector2(8, 0), 30, 10, 500, 0.05, o_horizontal);
    world.addRope(rope);
    world.addConstraint(new DoFConstraint(rope->points.front(), XY_AXIS, &(world.vars)));
    world.addConstraint(new DoFConstraint(rope->points.back(), XY_AXIS, &(world.vars)));*/

    // some simple axis constraints
    //DoFmotor* motor = new DoFmotor(world.bodies[4], MOVE_XY_ROTATE, &(world.vars));
    //world.addConstraint(motor);
    /*std::vector<double> motor_bias;
    motor_bias.push_back(-0.05);
    motor_bias.push_back(0.01);
    motor_bias.push_back(-0.1);
    motor->SetMotorBias(motor_bias);
    motor->SetMotorLimits(-1, 1);*/
}

void init_angry_circles()
{
    world_vars wvars = world.vars;
    wvars.GRAVITATION.v2 = -9.8;
    wvars.RESTITUTION = 0.3;
    wvars.FRICTION = 0.6;
    world.init();
    world.vars = wvars;

    // bounds
    double bigmass = 100000;
    //world.addBody(new Body(new rectangle(0, 14, 0, 2000, 3), bigmass, 0, 0, 0));
    world.addBody(new Body(new rectangle(0, -16, 0, 5000, 4), bigmass, 0, 0, 0));
    // bounds

    double mx = 25;
    // castle
    world.addBody(new Body(new rectangle(mx, -13, 0, 12, 2), 24, 0, 0, 0));
    world.addBody(new Body(new rectangle(mx, -10.5, 0, 2, 3), 6, 0, 0, 0));
    world.addBody(new Body(new rectangle(mx - 5, -10.5, 0, 2, 3), 6, 0, 0, 0));
    world.addBody(new Body(new rectangle(mx + 5, -10.5, 0, 2, 3), 6, 0, 0, 0));
    world.addBody(new Body(new rectangle(mx, -8.5, 0, 6, 1), 6, 0, 0, 0));
    world.addBody(new Body(new rectangle(mx - 4.5, -8, 0, 3, 2), 6, 0, 0, 0));
    world.addBody(new Body(new rectangle(mx + 4.5, -8, 0, 3, 2), 6, 0, 0, 0));
    world.addBody(new Body(new rectangle(mx - 4, -6, 0, 2, 2), 4, 0, 0, 0));
    world.addBody(new Body(new rectangle(mx + 4, -6, 0, 2, 2), 4, 0, 0, 0));
    world.addBody(new Body(new rectangle(mx, -4.5, 0, 9, 1), 9, 0, 0, 0));
    world.addBody(new Body(new rectangle(mx, -3.5, 0, 1, 1), 1, 0, 0, 0));
    world.addBody(new Body(new rectangle(mx - 4, -3.5, 0, 1, 1), 1, 0, 0, 0));
    world.addBody(new Body(new rectangle(mx + 4, -3.5, 0, 1, 1), 1, 0, 0, 0));
    
    // defenders
    world.addBody(new Body(new circle(mx - 2.5, -11.5, 0, 0.5), 0.5, 0, 0, 0));
    world.addBody(new Body(new circle(mx + 2.5, -11.5, 0, 0.5), 0.5, 0, 0, 0));
    world.addBody(new Body(new circle(mx, -7, 0, 1), 1, 0, 0, 0));

    mx = -21;
    // attackers
    world.addBody(new Body(new circle(mx, -13.5, 0, 0.5), 1, 0, 0, 0));
    world.addBody(new Body(new circle(mx - 2, -13, 0, 1), 2, 0, 0, 0));
    world.addBody(new Body(new circle(mx - 5, -12.5, 0, 1.5), 4, 0, 0, 0));

    mx = -15;
    // canon
    world.addBody(new Body(new rectangle(mx - 4.5, -12, 0, 2, 4), wvars.UNMOVABLE_MASS, 0, 0, 0));
    world.addBody(new Body(new rectangle(mx, -12.6, 0.25, 7, 1), wvars.UNMOVABLE_MASS, 0, 0, 0));
}

void init_many_rectangles()
{
    world_vars wvars = world.vars;
    wvars.GRAVITATION.v2 = 0;
    wvars.RESTITUTION = 0.5;
    wvars.FRICTION = 0.4;
    world.init();
    world.vars = wvars;
    double angle_vel = 0;

    int s = 12;
    double sp = (double)RAND_MAX / 12;
    for (int i = -s; i < s; i += 4)
    {
        for (int j = -s + 2; j < s; j += 4)
        {
            double r = rand() % 2 + 1;
            world.addBody(new Body(new rectangle(i, j, (double)rand() / sp, r, 3 - r),
                1, (double)rand() / sp - (double)rand() / sp,
            (double)rand() / sp - (double)rand() / sp, 0));
        }
    }

    // bounds
    double bigmass = 100000;
    world.addBody(new Body(new rectangle(-40, 0, 0, 20, 25), bigmass, 0, 0, 0));
    world.addBody(new Body(new rectangle(40, 0, 0, 20, 25), bigmass, 0, 0, 0));
    world.addBody(new Body(new rectangle(0, 14, 0, 200, 3), bigmass, 0, 0, 0));
    world.addBody(new Body(new rectangle(0, -14, 0, 200, 3), bigmass, 0, 0, 0));
    // bounds
    for (size_t i = 0; i < world.bodies.size(); i++)
    {
        int r = rand() % 10;
        if (r == 0 || r == 1)
            world.addConstraint(new DoFConstraint(world.bodies[i], XY_AXIS, &(world.vars)));
        if (r == 2 || r == 3)
            world.addConstraint(new DoFConstraint(world.bodies[i], ANGLE, &(world.vars)));
    }
}

void init_many_circles()
{
    world_vars wvars = world.vars;
    wvars.GRAVITATION.v2 = 0;
    wvars.RESTITUTION = 0.8;
    wvars.FRICTION = 0.1;
    world.init();
    world.vars = wvars;
    double angle_vel = 0;

    int s = 12;
    double sp = (double)RAND_MAX / 12;
    for (int i = 0; i < 2 * s; i += 4)
    {
        for (int j = -s + 2; j < s; j += 4)
        {
            double r = rand() % 2 + 1;
            world.addBody(new Body(new circle(i, j, (double)rand(), 1), 3.14,
                (double)rand() / sp - (double)rand() / sp,
                (double)rand() / sp - (double)rand() / sp, 0));
        }
    }

    Rope* rope = new Rope(Vector2(-3, 12), 55, 10, 500, 0.05, o_vertical);
    world.addRope(rope);
    world.addConstraint(new DoFConstraint(rope->points.front(), XY_AXIS, &(world.vars)));
    world.addConstraint(new DoFConstraint(rope->points.back(), XY_AXIS, &(world.vars)));

    // bounds
    double bigmass = 100000;
    world.addBody(new Body(new rectangle(-40, 0, 0, 20, 25), bigmass, 0, 0, 0));
    world.addBody(new Body(new rectangle(40, 0, 0, 20, 25), bigmass, 0, 0, 0));
    world.addBody(new Body(new rectangle(0, 14, 0, 200, 3), bigmass, 0, 0, 0));
    world.addBody(new Body(new rectangle(0, -14, 0, 200, 3), bigmass, 0, 0, 0));
    // bounds
}


void stack_init()
{
    world.init();

    double angle_vel = 0;
    std::vector<Body*> bodies;

    Body* body;// = Body(&(rectangle()), 0, 0, 0, 0);
    rectangle* rect;

    // lower bound
    rect = new rectangle(0, -14, 0, 200, 4);
    body = new Body(rect, 100000, 0, 0, 0);
    bodies.push_back(body);
    delete rect;

    rect = new rectangle(0, -10, 0, 4, 4);
    body = new Body(rect, 16, 0, 0, 0);
    bodies.push_back(body);
    delete rect;

    rect = new rectangle(0, -7, 0, 8, 2);
    body = new Body(rect, 16, 0, 0, 0);
    bodies.push_back(body);
    delete rect;

    rect = new rectangle(0, -3, 0, 2, 6);
    body = new Body(rect, 12, 0, 0, 0);
    bodies.push_back(body);
    delete rect;

    rect = new rectangle(0.5, 2, 0, 4, 4);
    body = new Body(rect, 12, 0, 0, 0);
    bodies.push_back(body);
    delete rect;

    for (size_t nb = 0; nb < bodies.size(); nb++)
    {
        world.addBody(bodies[nb]);
    }
}
