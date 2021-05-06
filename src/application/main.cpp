#include "CGL/CGL.h"
#include "CGL/viewer.h"

#define TINYEXR_IMPLEMENTATION
#include "CGL/tinyexr.h"

#include "application.h"
typedef uint32_t gid_t;
#include "util/image.h"
typedef uint32_t gid_t;

#include <iostream>
#ifdef _WIN32
#include "util/win32/getopt.h"
#else
#include <unistd.h>
#endif

// begin integration
#include "CGL/misc.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"
#include "CGL/matrix3x3.h"
// end integration

using namespace std;
using namespace CGL;

#define msg(s) cerr << "[PathTracer] " << s << endl;

void usage(const char* binaryName) {
  printf("Usage: %s [options] <scenefile>\n", binaryName);
  printf("Program Options:\n");
  printf("  -s  <INT>        Number of camera rays per pixel\n");
  printf("  -l  <INT>        Number of samples per area light\n");
  printf("  -t  <INT>        Number of render threads\n");
  printf("  -m  <INT>        Maximum ray depth\n");
  printf("  -e  <PATH>       Path to environment map\n");
  printf("  -b  <FLOAT>      The size of the aperture\n");
  printf("  -d  <FLOAT>      The focal distance\n");
  printf("  -f  <FILENAME>   Image (.png) file to save output to in windowless mode\n");
  printf("  -r  <INT> <INT>  Width and height of output image (if windowless)\n");
  printf("  -h               Print this help message\n");
  printf("\n");
}

HDRImageBuffer* load_exr(const char* file_path) {
  
  const char* err;
  
  EXRImage exr;
  InitEXRImage(&exr);

  int ret = ParseMultiChannelEXRHeaderFromFile(&exr, file_path, &err);
  if (ret != 0) {
    msg("Error parsing OpenEXR file: " << err);
    return NULL;
  }

  for (int i = 0; i < exr.num_channels; i++) {
    if (exr.pixel_types[i] == TINYEXR_PIXELTYPE_HALF) {
      exr.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
    }
  }

  ret = LoadMultiChannelEXRFromFile(&exr, file_path, &err);
  if (ret != 0) {
    msg("Error loading OpenEXR file: " << err);
    exit(EXIT_FAILURE);
  }

  HDRImageBuffer* envmap = new HDRImageBuffer();
  envmap->resize(exr.width, exr.height);
  float* channel_r = (float*) exr.images[2];
  float* channel_g = (float*) exr.images[1];
  float* channel_b = (float*) exr.images[0];
  for (size_t i = 0; i < exr.width * exr.height; i++) {
    envmap->data[i] = Vector3D(channel_r[i], 
                               channel_g[i], 
                               channel_b[i]);
  }

  return envmap;
}

// begin integration
Matrix3x3 make_2_matrix(float a, float b, float c, float d) {
    // | a b 0 |
    // | c d 0 |
    // | 0 0 0 |
    return Matrix3x3(a, b, 0, c, d, 0, 0, 0, 0);
}

// a 2x2 inversion on a zero-padded 3x3 matrix
Matrix3x3 invert2x2(Matrix3x3 mat) {
    float a = mat(0, 0);
    float b = mat(0, 1);
    float c = mat(1, 0);
    float d = mat(1, 1);
    return 1.0 / (a*d - b*c) * make_2_matrix(d, -b, -c, a);
}

Matrix3x3 T(float d) {
    return make_2_matrix(1, d, 0, 1);
}

Matrix3x3 generate_R(float c, float n1, float n2) {
    return make_2_matrix(1, 0, c*(n1-n2)/n2, n1/n2);
}

Matrix3x3 generate_L(float c) {
    return make_2_matrix(1, 0, 2*c, 1);
}

float pupil_height = 12.3;

Matrix3x3 Ts[] = {
    T(7.700),  // 1
    T(1.850),
    T(3.520),
    T(1.850),
    T(4.180),
    T(3.000),
    T(1.850),
    T(7.270),
    T(83.91)
};

float red_refr[] = {1.652, 1.5991, 1, 1.6396, 1, 1, 1.5776, 1.68990, 1};
float green_refr[] = {1.652, 1.6113, 1, 1.65, 1, 1, 1.5885, 1.6999, 1};
float blue_refr[] = {1.652, 1.6164, 1, 1.6542, 1, 1, 1.5930, 1.7040, 1};
float curvatures[] = {1/30.810, 1/-89.350, 1/580.380, 1/-80.630, 1/28.340, 0, 0, 1/32.190, 1/-52.990, 1/81.320};

// assumes color_
std::vector<Matrix3x3> create_Rs_for_color(float color_refr[]) {
    std::vector<Matrix3x3> arr;
    float prev_n = 1.00;
    // NOTE: hard coded array length for 9 lenses!
    for (int i = 0; i < 9; i++) {
        arr.push_back(generate_R(curvatures[i], prev_n, color_refr[i]));
        prev_n = color_refr[i];
    }
    return arr;
}

std::vector<Matrix3x3> create_Ls() {
    std::vector<Matrix3x3> arr;
    for (int i = 0; i < 9; i++) {
        arr.push_back(generate_L(curvatures[i]));
    }
    return arr;
}

float radii_size(float r1, float r2) {
    return (r1 - r2) * (r1 - r2);
}

std::vector<Matrix3x3> Ls = create_Ls();

std::vector<Matrix3x3> R_red = create_Rs_for_color(red_refr);
std::vector<Matrix3x3> R_blue = create_Rs_for_color(blue_refr);
std::vector<Matrix3x3> R_green = create_Rs_for_color(green_refr);

Vector2D trace_ray_auto(float r, float theta, int i, int j, std::vector<Matrix3x3> color_R) {
    // mapping to 3D
    Vector3D ray = Vector3D(r, theta, 0);
    
    int mini = std::min(i, j);
    int maxj = std::max(i, j);
    i = mini;
    j = maxj;
    
    Matrix3x3 M = make_2_matrix(1, 0, 0, 1);
    
    // forward through j-1
    for (int k = 0; k < j; k++)
        M = Ts[k] * color_R[k] * M;
    
    // reflect off of Lj
    M = Ls[j] * M;
    
    // cout << "-1. no inv alr" << endl << M << endl;
    
    for (int k = i + 1; k > j - 2; k--)
        M = invert2x2(color_R[k]) * Ts[k] * M;
    
    // cout << "0. inv alr" << endl << M << endl;
    
    M = Ts[i] * invert2x2(Ls[i]) * Ts[i] * M;
    
    // cout << "1." << endl << M << endl;
    
    // zero indexed
    for (int k = i+1; k < 9; k++) {
        if (k == 5) {
            Vector3D after_ap = M * ray;
            if (after_ap.x > 11.6 || after_ap.x < -11.6) {
                cout << "recasting, got aperature status: " << after_ap.x << endl;
                float r_a = 11.6;
                if (r < 0)
                    r_a = -11.5;
                float r_e = (r_a - M(0, 1) * theta) / M(0, 0);
                ray = Vector3D(r_e, theta, 0);
                cout << "ray we're casting: " << ray << endl;
            }
            // crossing the aperture
            M = Ts[k] * M;
            continue;
        }
        M = Ts[k] * color_R[k] * M;
    }
    
    // cout << "2." << endl << M << endl;
    
    Vector3D res = M * ray;
    return Vector2D(res.x, res.y);
}

void trace_all_rays_auto() {
    // for all i->j from 0->4 (inclusive) all pairs
    for (int i = 0; i < 5; i++) {
        for (int j = i+1; j < 5; j++) {
            trace_ray_auto(1, 0.01, i, j, R_red);
            trace_ray_auto(1, 0.01, i, j, R_green);
            trace_ray_auto(1, 0.01, i, j, R_blue);
        }
    }
}


// end integration

int main( int argc, char** argv ) {
    
  // testing
    /*
  cout << trace_ray_auto(14.5, 0.025, 1, 3, R_red) << endl;
  cout << trace_ray_auto(14.5, 0.025, 1, 3, R_green) << endl;
  cout << trace_ray_auto(14.5, 0.025, 1, 3, R_blue) << endl;
    
  cout << trace_ray_auto(-14.5, 0.025, 1, 3, R_red) << endl;
  cout << trace_ray_auto(-14.5, 0.025, 1, 3, R_green) << endl;
  cout << trace_ray_auto(-14.5, 0.025, 1, 3, R_blue) << endl;
   
  cout << trace_ray_auto(1000, 0.05, 2, 4, R_blue) << endl;
  cout << trace_ray_auto(-1000, 0.05, 1, 3, R_blue) << endl;
     */

  // get the options
  AppConfig config; int opt;
  bool write_to_file = false;
  size_t w = 0, h = 0, x = -1, y = 0, dx = 0, dy = 0;
  string filename, cam_settings = "";
  while ( (opt = getopt(argc, argv, "s:l:t:m:e:h:H:f:r:c:b:d:a:p:")) != -1 ) {  // for each option...
    switch ( opt ) {
    case 'f':
      write_to_file = true;
      filename  = string(optarg);
      break;
    case 'r':
      w = atoi(argv[optind-1]);
      h = atoi(argv[optind]);
      optind++;
      break;
    case 'p':
      x = atoi(argv[optind-1]);
      y = atoi(argv[optind-0]);
      dx = atoi(argv[optind+1]);
      dy = atoi(argv[optind+2]);
      optind += 3;
      break;
    case 's':
      config.pathtracer_ns_aa = atoi(optarg);
      break;
    case 'l':
      config.pathtracer_ns_area_light = atoi(optarg);
      break;
    case 't':
      config.pathtracer_num_threads = atoi(optarg);
      break;
    case 'm':
      config.pathtracer_max_ray_depth = atoi(optarg);
      break;
    case 'e':
      std::cout << "[PathTracer] Loading environment map " << optarg << std::endl;
      config.pathtracer_envmap = load_exr(optarg);
      break;
    case 'c':
      cam_settings = string(optarg);
      break;
    case 'b':
      config.pathtracer_lensRadius = atof(optarg);
      break;
    case 'd':
      config.pathtracer_focalDistance = atof(optarg);
      break;
    case 'a':
      config.pathtracer_samples_per_patch = atoi(argv[optind-1]);
      config.pathtracer_max_tolerance = atof(argv[optind]);
      optind++;
      break;
    case 'H':
      config.pathtracer_direct_hemisphere_sample = true;
      optind--;
      break;
    default:
      usage(argv[0]);
      return 1;
    }
  }

  // print usage if no argument given
  if (optind >= argc) {
    usage(argv[0]);
    return 1;
  }

  string sceneFilePath = argv[optind];
  msg("Input scene file: " << sceneFilePath);
  string sceneFile = sceneFilePath.substr(sceneFilePath.find_last_of('/')+1);
  sceneFile = sceneFile.substr(0,sceneFile.find(".dae"));
  config.pathtracer_filename = sceneFile;

  // parse scene
  Collada::SceneInfo *sceneInfo = new Collada::SceneInfo();
  if (Collada::ColladaParser::load(sceneFilePath.c_str(), sceneInfo) < 0) {
    delete sceneInfo;
    exit(0);
  }

  // create application
  Application *app  = new Application(config, !write_to_file);

  msg("Rendering using " << config.pathtracer_num_threads << " threads");

  // write straight to file without opening a window if -f option provided
  if (write_to_file) {
    app->init();
    app->load(sceneInfo);
    delete sceneInfo;

    if (w && h)
      app->resize(w, h);

    if (cam_settings != "")
      app->load_camera(cam_settings);

    app->render_to_file(filename, x, y, dx, dy);
    return 0;
  }

  // create viewer
  Viewer viewer = Viewer();

  // set renderer
  viewer.set_renderer(app);

  // init viewer
  viewer.init();

  // load scene 
  app->load(sceneInfo);

  delete sceneInfo;

  if (w && h)
    viewer.resize(w, h);
    
  if (cam_settings != "")
    app->load_camera(cam_settings);

  // start viewer
  viewer.start();

  return 0;

}
