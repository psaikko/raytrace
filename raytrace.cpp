// Include standard headers
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <thread>
#include <vector>
#include <cmath>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
using namespace glm;

#define EPS 1e-11
const int N_RANDOMS = 611953;
const int WINDOW_W = 640;
const int WINDOW_H = 480;
const bool SOFT_SHADOWS = false;
const bool SAMPLING = false;
const int N_SAMPLES = 1;
const int N_THREADS = 2;
const bool DOF = false;
const double DOF_STRENGTH = 99.0;
const double FOC_DIST = 16.0; //16.0f;
const bool WALLS = false;

const int MAXDEPTH = 5;
const dvec3 DARK_GRAY(0,0,0);
const dvec3 NO_LIGHT(0,0,0);

dvec3 selfPos(15,8,15);
dvec3 direction(-sqrt(.3f), 0, -sqrt(.7f));

double R[N_RANDOMS];
// random double [0, 1]
inline double randd() {
  static unsigned i = 0;
  return R[++i % N_RANDOMS];
}

enum Material { DIFFUSE, REFLECTIVE, REFRACTIVE };

struct isect {
  double   dist;
  dvec3    ref, color, light;
  Material mat;
  isect(){}
  isect(double d, dvec3 r, dvec3 c, dvec3 l, Material m) {
    dist  = d;
    ref   = r;
    color = c;
    light = l;
    mat   = m;
  }
};

struct sphere {
  dvec3 p, c, l;
  double r;
  Material m;
  sphere(){}
  sphere(dvec3 pos, double rad, dvec3 color, dvec3 light, Material mat) {
    p = pos;    
    r = rad;
    c = color;
    l = light;
    m = mat;
  }
  void intersect(const dvec3& src, const dvec3& dir, isect& is);
};

struct triangle {
  dvec3 p1, p2, p3, c, l;
  Material m;
  triangle(){}
  triangle(dvec3 pt1, dvec3 pt2, dvec3 pt3, dvec3 color, dvec3 light, Material mat) {
    p1 = pt1; p2 = pt2; p3 = pt3;
    c = color;
    l = light;
    m = mat;
  }
  void intersect(const dvec3& src, const dvec3& dir, isect& is);
};

void sphere::intersect(const dvec3& src, const dvec3& dir, isect& is) { 
  dvec3 ray_tr = src - p;
  // B, C quadratic coefficients (A = 1 from normalized dir)
  double B    = dot(ray_tr, dir), // actually B/2
         C    = dot(ray_tr, ray_tr) - r * r,
         disc = B*B - C; 
  if (disc > EPS) {
    // ray intersects sphere
    double d = -B - sqrt(disc);
    if (d > EPS && d < is.dist) {
      is.dist  = d;
      is.ref   = normalize(ray_tr + dir * d);
      is.color = c;
      is.light = l;
      is.mat   = m;
    }
  }
}

void triangle::intersect(const dvec3& src, const dvec3& dir, isect& is) {
  // Möller-Trumbore algorithm
  // for triangle collision
  dvec3 e1 = p2 - p1,
        e2 = p3 - p1,                   // Find vectors for two edges sharing V1
        P  = cross(dir, e2);            // Begin calculating determinant - also used to calculate u parameter
  double det = dot(e1, P);              // det ~ 0 : ray lies in plane of triangle
  if(det > -EPS && det < EPS) return;   // NOT CULLING
  double inv_det = 1.0 / det;
  dvec3  T = src - p1    ;              // calculate distance from V1 to ray origin
  double u = dot(T, P) * inv_det;       // Calculate u parameter and test bound
  if(u < 0.0 || u > 1.0) return;        // The intersection lies outside of the triangle
  dvec3  Q = cross(T, e1);              // Prepare to test v parameter
  double v = dot(dir, Q) * inv_det;     // Calculate V parameter and test bound
  if(v < 0.0 || u + v > 1.0) return;    // The intersection lies outside of the triangle
  double d = dot(e2, Q) * inv_det;
  if(d > EPS && d < is.dist) {          // Ray intersection
    // dvec3 N = cross(e1, e2);//normalize(cross(e1, e2));
    // ref = normalize(2.0 * N * dot(dir, N) - dir);
    is.dist  = d;
    is.ref   = -normalize(cross(e1, e2));
    is.color = c;
    is.light = l;
    is.mat   = m;
  }
}

std::vector<sphere>   lights;
std::vector<sphere>   spheres;
std::vector<triangle> triangles;

void rt_init(void) {

  for (int i = 0; i < N_RANDOMS; ++i)
    R[i] = (double)rand() / RAND_MAX;
  
  float r = 5.0; // radius
  float h = 20.0; // height
  float c = 20.0; // sphere count
  float t = 2.0;  // turns

  for (float i = 0; i < c; ++i) {
    spheres.emplace_back(dvec3(r*sin(2*M_PI*t*(i/c)), 
                               h*i/c, 
                               r*cos(2*M_PI*t*(i/c))),
                         1.0,
                         dvec3(1.0,1.0,1.0),
                         NO_LIGHT,				
                         REFLECTIVE);
    spheres.emplace_back(dvec3(r*sin(2*M_PI*t*(i/c) + M_PI), 
                               h*i/c, 
                               r*cos(2*M_PI*t*(i/c) + M_PI)),
                         1.0,
                         dvec3(1.0,1.0,1.0),
                         NO_LIGHT,				
                         REFLECTIVE);
  }

  spheres.emplace_back(dvec3(0, 7.5, 0), 1.5, dvec3(1.0,1.0,1.0), NO_LIGHT, REFRACTIVE);

  // light
  lights.emplace_back(dvec3(  -9,  10,  -9), 
                      0.5, 
                      dvec3( 0.5, 0.5, 0.5), 
                      dvec3(   1, 0.5,   1),
                      REFLECTIVE);
  lights.emplace_back(dvec3(   9,  10,  -9), 
                      0.5, 
                      dvec3( 0.5, 0.5, 0.5), 
                      dvec3(   1,   1, 0.5),
                      REFLECTIVE);
  lights.emplace_back(dvec3(  -9,  10,   9), 
                      0.5, 
                      dvec3( 0.5, 0.5, 0.5), 
                      dvec3( 0.5,   1,   1),
                      REFLECTIVE);
  lights.emplace_back(dvec3(   9,  10,   9), 
                      0.5, 
                      dvec3( 0.5, 0.5, 0.5), 
                      dvec3( 0.5, 0.5,   1),
                      REFLECTIVE);
  
  if (WALLS) {
  double wc = 0.7;
    triangles.emplace_back(dvec3( -20,   0, -20),
                           dvec3(  20,   0, -20),
                           dvec3(  20,  30, -20),
                           dvec3(  wc,  wc,  wc),
                           NO_LIGHT,
                           REFLECTIVE);
    triangles.emplace_back(dvec3( -20,   0, -20),
                           dvec3(  20,  30, -20),
                           dvec3( -20,  30, -20),
                           dvec3(  wc,  wc,  wc),
                           NO_LIGHT,
                           REFLECTIVE);
    triangles.emplace_back(dvec3(-20,0,-20), 
                           dvec3(-20,0,20), 
                           dvec3(-20,30,20),
                           dvec3(  wc,  wc,  wc),
                           NO_LIGHT,
                           REFLECTIVE);
    triangles.emplace_back(dvec3(-20,0,-20), 
                           dvec3(-20,30,20), 
                           dvec3(-20,30,-20),
                           dvec3(  wc,  wc,  wc),
                           NO_LIGHT,
                           REFLECTIVE);
    triangles.emplace_back(dvec3(20,0,-20), 
                           dvec3(20,0,20), 
                           dvec3(20,30,20),
                           dvec3(  wc,  wc,  wc),
                           NO_LIGHT,
                           REFLECTIVE);
    triangles.emplace_back(dvec3(20,0,-20), 
                           dvec3(20,30,20), 
                           dvec3(20,30,-20),
                           dvec3(  wc,  wc,  wc),
                           NO_LIGHT,
                           REFLECTIVE);
    triangles.emplace_back(dvec3(-20,0,20), 
                           dvec3(20,0,20), 
                           dvec3(20,30,20),
                           dvec3(  wc,  wc,  wc),
                           NO_LIGHT,
                           REFLECTIVE);
    triangles.emplace_back(dvec3(-20,0,20), 
                           dvec3(20,30,20), 
                           dvec3(-20,30,20),
                           dvec3(  wc,  wc,  wc),
                           NO_LIGHT,
                           REFLECTIVE);
  }
  // floor
  triangles.emplace_back(dvec3( -1000,   0, -1000),
                         dvec3(  1000,   0, -1000),
                         dvec3( -1000,   0,  1000),
                         dvec3(   0.6, 0.7,   0.6),
                         NO_LIGHT,
                         DIFFUSE);
  triangles.emplace_back(dvec3(  1000,   0,  1000),
                         dvec3( -1000,   0,  1000),
                         dvec3(  1000,   0, -1000),
                         dvec3(   0.6, 0.7,   0.6),
                         NO_LIGHT,
                         DIFFUSE);
}

void trace(const dvec3& src, const dvec3& dir, isect& is) {
  is.light = dvec3(0,0,0);
  is.color = dvec3(.7, .6, 1.0) * (double)pow(dir.y, 2);
  is.dist  = 1e10;

  // sphere collisions
  for (sphere sph : spheres)
    sph.intersect(src, dir, is);

  // sphere collisions
  for (sphere light : lights)
    light.intersect(src, dir, is);

  // triangle collisions
  for (triangle tri : triangles)
    tri.intersect(src, dir, is);
}

// returns a color!
dvec3 sample(const dvec3& src, const dvec3& dir, int d = 0) {

  if (d > MAXDEPTH) return dvec3(0,0,0);

  isect is;
  trace(src, dir, is);

  if (is.dist == 1e10) return is.color;
  
  // intersection coord
  dvec3 intersect = dvec3(src + dir * is.dist),        
        r         = dvec3(dir + is.ref * dot(is.ref, dir) * -2.0);

  if (is.mat == DIFFUSE) {
    dvec3 obj_color = is.color;
    dvec3 obj_ref   = is.ref;
    dvec3 c(0,0,0);
    for (sphere light : lights) {
      double lx = light.p.x + SOFT_SHADOWS * randd(),
             ly = light.p.y,
             lz = light.p.z + SOFT_SHADOWS * randd();
      // direction to light
      dvec3 lightdir = normalize(dvec3(lx, ly, lz) - intersect);

      dvec3 lc = obj_color;

      // lambertian factor
      double b = dot(lightdir, obj_ref);
      if (b < 0) continue;
      lc *= pow(b, 3);

      // apply light color
      trace(intersect, lightdir, is);
      lc *= is.light;
      c += lc;
    }
    return clamp(c, 0.0, 1.0);
  } else if (is.light != dvec3(0,0,0)) {
    return is.light;
  } else if (is.mat == REFLECTIVE) {
    return clamp(sample(intersect, r, d+1) * is.color, 0.0, 1.0);
  } else { // is.mat == REFRACTIVE
    // calculate refraction
    double n = 0.9;
    double cosI = -dot(is.ref, dir);
    double cosT2 = 1.0 - n * n * (1.0 - cosI * cosI);
    if (cosT2 > 0.0f) {
      dvec3 T = (n * dir) + is.ref * (n * cosI - sqrt( cosT2 ));
      return sample(intersect, T, d+1);
    } else {
      return is.color;
    }
  }
}


void renderline(const int y, char* const data, const dvec3& up, const dvec3& rt,
                const dvec3& c, const dvec3& src) 
{

  const double colorscale = 255.0 / (double)N_SAMPLES;

  for (int x = 0; x < WINDOW_W; ++x) {
    dvec3 p(DARK_GRAY);
    // sample n rays with random offsets
    for (int r = 0; r < N_SAMPLES; ++r) {
      // lens pertubation
      dvec3 t =
          DOF ? (up * (randd() - 0.5) + rt * (randd() - 0.5)) * DOF_STRENGTH
              : dvec3(0, 0, 0);

      double xoff = SAMPLING ? randd() : 0.5,
             yoff = SAMPLING ? randd() : 0.5;
      p += sample(src + t,
                  normalize(t * -1.0 + (up * (xoff + x) + rt * (yoff + y) + c) *
                                           FOC_DIST));
    }

    p *= colorscale;

    int t = y * WINDOW_W + x;
    data[t * 3 + 0] = p.x;
    data[t * 3 + 1] = p.y;
    data[t * 3 + 2] = p.z;
  }
}


void raytrace(char *data) {

  dvec3 g = direction,                   // camera direction
       up = normalize(cross(dvec3(0, 1, 0), g)) / (double)WINDOW_H,  // up vector
       rt = normalize(cross(g, up)) / (double)WINDOW_W,             // right vector
        c = up*(-WINDOW_H/2.0) + rt*(-WINDOW_W/2.0) + g, // corner of focal plane
      src = selfPos;                             // ray source (eye pos)

  // for each row of pixels
  std::thread t[N_THREADS];
  for(int y = 0; y < WINDOW_H; y += N_THREADS) {
    for (int i = 0; i < N_THREADS; ++i)
      t[i] = std::thread(renderline, y+i, data, up, rt, c, src);
    for (int i = 0; i < N_THREADS; ++i)
      t[i].join();
  }
}

//
// end raytracer
//

int main(int argc, char ** argv) {

  bool still = argc == 2;

  rt_init();

  // Initialise GLFW
  if (!glfwInit()) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    return -1;
  }

  char data[WINDOW_H*WINDOW_W*3];

  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  //glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  // Open a window and create its OpenGL context
  window = glfwCreateWindow(WINDOW_W, WINDOW_H, "raytracer", NULL, NULL);

  glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

  glfwMakeContextCurrent(window);

  glewExperimental = GL_TRUE;
  glewInit();

  float a = 3.1415;
  bool rotate = true;

  //raytrace(data);

  long frame = 0;
  while (++frame) {
    if (rotate) a += 0.05f;
    lights[0].p.x = (double)14 * sin(a);
    lights[0].p.z = (double)14 * cos(a);
    lights[1].p.x = (double)14 * sin(a - 3.14 / 2);
    lights[1].p.z = (double)14 * cos(a - 3.14 / 2);
    lights[2].p.x = (double)14 * sin(a + 3.14 / 2);
    lights[2].p.z = (double)14 * cos(a + 3.14 / 2);
    lights[3].p.x = (double)14 * sin(a + 3.14);
    lights[3].p.z = (double)14 * cos(a + 3.14);

    if (!still || frame == 1) {
      glClear( GL_COLOR_BUFFER_BIT );
      glClearColor(0.0f, 0.0f, 0.4f, 0.5f);
      raytrace(data); 
      glDrawPixels(WINDOW_W, WINDOW_H, GL_RGB, GL_UNSIGNED_BYTE, data);
      // Swap buffers
      glfwSwapBuffers(window);
    }

    glfwPollEvents();
    if (glfwGetKey(window, GLFW_KEY_KP_SUBTRACT) == GLFW_PRESS)
      rotate = false;
    if (glfwGetKey(window, GLFW_KEY_KP_ADD) == GLFW_PRESS)
      rotate = true;
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
      selfPos += 0.7*direction;
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
      selfPos -= 0.7*direction;
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
      direction = dmat3( cos(0.05f),  0.f, sin(0.05f),
                                0.f,  1.f,        0.f,
                        -sin(0.05f),  0.f, cos(0.05f)) * direction;
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
      direction = dmat3( cos(-0.05f),  0.f, sin(-0.05f),
                                 0.f,  1.f,         0.f,
                        -sin(-0.05f),  0.f, cos(-0.05f)) * direction;
    // Check if the ESC key was pressed or the window was closed
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS ||
        glfwWindowShouldClose(window) == 1)
      break;
  }

  glfwTerminate();
  return 0;
}
