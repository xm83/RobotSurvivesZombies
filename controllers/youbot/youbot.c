/*
 * Copyright 1996-2019 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Starts with a predefined behaviors and then
 *                read the user keyboard inputs to actuate the
 *                robot
 */



#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/camera.h>
#include <webots/accelerometer.h>
#include <webots/lidar.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/range_finder.h>
#include <webots/gyro.h>
#include <webots/light_sensor.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>

#include <youbot_zombie_1.h>


int robot_angle = 0;
#define TIME_STEP 32

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// ONLY USE THE FOLLOWING FUNCTIONS TO MOVE THE ROBOT /////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


void stop()
{
  base_reset();
}

void go_forward()
{
  base_forwards();
}

void go_backward()
{
  base_backwards();
}

void turn_left()
{
  base_turn_left();
  robot_angle = robot_angle + 90;
  if (robot_angle == 360)
    robot_angle = 0;

}

void turn_right()
{
  base_turn_right();
  robot_angle = robot_angle - 90;
  if (robot_angle == -90)
    robot_angle = 270;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
#define OBJ_TYPES 12
#define max(a,b) (a>b?a:b)
#define min(a,b) (a<b?a:b)

// weights assigned to each object type; 
// c_curDir helps prioritize zombie keeps moving forward / backward instead of turning
double c[] = {-2.5, -4, -3, -2, 1, 1, 1, 1, -10, 0.1, 2, -10}, c_curDir = 1000;

typedef enum{AquaZombie, PurpleZombie, BlueZombie, GreenZombie, RedBerry, YellowBerry, OrangeBerry, PinkBerry, Wall, Stump, Tree, BoundaryWall} TYPE;

typedef struct objCoords {
   TYPE type;
   int left;
   int right;
   int bottom;
   int dir;
   int dist;

} Coords;

// returned by robot_control
typedef struct robotControlResult {
  int global_best_dir;
  int local_best_dir;
  bool nearStump;
} robotControlResult;

// returned by parse_camera
typedef struct cameraObjResult {
  Coords * objs_ptr;
  int len;
} CameraResult;

typedef struct {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
} hsv;

typedef struct {
    double r;       // angle in degrees
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
} rgb;


// referenced from https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
rgb hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;
}

// referenced from https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
// return Hue value
hsv rgb2hsv(int r, int g, int b)
{
    hsv     out;
    double  min, max, delta;

    min = r < g ? r : g;
    min = min  < b ? min  : b;

    max = r > g ? r : g;
    max = max  > b ? max  : b;

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0
        // s = 0, h is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    if( r >= max )                           // > is bogus, just keeps compilor happy
        out.h = ( g - b ) / delta;        // between yellow & magenta
    else
    if( g >= max )
        out.h = 2.0 + ( b - r ) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + ( r - g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}

// check which type a hsv color belongs to
TYPE checkColor(hsv c)
{
  rgb RGB = hsv2rgb(c);
  // ground color
  if (210<=RGB.r && RGB.r<=225 && 160 <= RGB.g && RGB.g <= 210 && 150<=RGB.b && RGB.b<=210)
    return -1;
  else if (228 == c.h && 0.02 <= c.s && c.s <= 0.03 && c.v == 212)
    return Wall;
  else if (226 <= c.h && c.h <= 227 && 0.27 <= c.s && c.s <= 0.28 && c.v == 95)
    return Wall;
  else if (225 == c.h && 0.28 <= c.s && c.s <= 0.29 && c.v == 14)
    return Stump;
  // else if (c.s < 0.35 && c.v < 60)
  //   return Tree;
  else if (0 <= c.h && c.h <= 8 && c.s >= 0.4)
   return RedBerry;
  else if (54 <= c.h && c.h <= 59)
   return YellowBerry;
  else if (11 <= c.h && c.h <= 24 && 0.5 <= c.s && c.s <= 0.6)// && 195 <= c.v && c.v <= 200)
   return OrangeBerry;
  else if (287 <= c.h && c.h <= 325)
   return PinkBerry;
  // Zombies
  else if (165 <= c.h && c.h <= 180 && c.s >= 0.2)
    return AquaZombie;
  else if (265 <= c.h && c.h <= 285 && c.s >= 0.4)
    return PurpleZombie;
  else if (205 <= c.h && c.h <= 225 && RGB.r <=30)
    return BlueZombie;
  else if (115 <= c.h && c.h <= 125)
    return GreenZombie;
  else if (0.06 <= c.s && c.s <= 0.18 && c.v <= 78)
    return BoundaryWall;
  else
    return -1;
}

// depth first search for connected same-color objects
void dfs(int w, int h, hsv array[w][h], int x, int y, TYPE color, Coords * objectTracker, int objCount)
{
    if (array[x][y].h == -1)
      return;

    array[x][y].h = -1;
    objectTracker[objCount].left = min(objectTracker[objCount].left, x);
    objectTracker[objCount].right = max(objectTracker[objCount].right, x);
    objectTracker[objCount].bottom = max(objectTracker[objCount].bottom, y);
    //printf("%d %d", x, y);
    if (x + 1 < w && checkColor(array[x + 1][y]) == color)
      dfs(w, h, array, x + 1, y, color, objectTracker, objCount);
    if (y + 1 < h && checkColor(array[x][y + 1]) == color)
      dfs(w, h, array, x, y + 1, color, objectTracker, objCount);
    if (x - 1 > -1 && checkColor(array[x - 1][y]) == color)
      dfs(w, h, array, x - 1, y, color, objectTracker, objCount);
    if (y - 1 > -1 && checkColor(array[x][y - 1]) == color)
      dfs(w, h, array, x, y - 1, color, objectTracker, objCount);
}

// check if a color corresponds to wall or ground
int isWallorGround(int r, int b, int g)
{
  // ground beige -- (215,178,163) | (207,165,152) | (208,169,154) | (206,155,137) | (215,170,152)
  if (205 <= r && r <= 215 && 150 <= b && b <= 170 && 135 <= g && g <= 165)
    return 1;
  // wall (212,213,216) || (69,75,95)
  if ((r == 212 && g == 213 && r == 216) || (r == 69 && g == 75 && r == 95))
    return 1;

  return 0;
}

// check if certain detection of objects can be merged into one object
int checkMerge(Coords * objtks, Coords cur, int objCount){
    for (int k=0;k<objCount;k++){
                if (objtks[k].type!=cur.type || objtks[k].left > cur.right || objtks[k].right < cur.left){
                     continue;
                }
                objtks[k].left = min(objtks[k].left, cur.left);
                objtks[k].right = max(objtks[k].right, cur.right);
                return k;

    }
    return -1;
}

// parse the camera image
CameraResult parse_camera(const unsigned char* image, int width) {
  int r, g, b;
  hsv hueEncodedImage[width][width/2];
    for (int y = 0; y < width/2; y++) {
    for (int x = 0; x < width; x++) {
      r = wb_camera_image_get_red(image, width, x, y);
      g = wb_camera_image_get_green(image, width, x, y);
      b = wb_camera_image_get_blue(image, width, x, y);
      hueEncodedImage[x][y] = rgb2hsv(r, g, b);
      }
    }

    TYPE t;

    int objCount = 0;
    int totalNumOfObjs = OBJ_TYPES * 2;
    Coords * objectTracker = calloc(totalNumOfObjs, sizeof(Coords));
    for (int i = 0; i < totalNumOfObjs; i++)
    {
        objectTracker[i].left = 300; // rightmost val
        objectTracker[i].right = -1; // leftmost val
        objectTracker[i].bottom = 0; // topmost val
    }


    for (int y = 0; y < width/2; y++)
    {
        for (int x = 0; x < width; x++)
        {
            // if is one of the relevant colors
            if (hueEncodedImage[x][y].h != -1)
            {
              t = checkColor(hueEncodedImage[x][y]);
              if ((t != -1 && !isWallorGround(r, g, b)))
              // || // extra condition allowing stumps with low v
              // (225 == hueEncodedImage[x][y].h && 0.28 <= hueEncodedImage[x][y].s
              // && hueEncodedImage[x][y].s <= 0.29 && hueEncodedImage[x][y].v == 14))// &&    (hueEncodedImage[x][y].s < 0.35 && hueEncodedImage[x][y].v < 60)
              {
                 dfs(width, width/2, hueEncodedImage, x, y, t, objectTracker, objCount);
                 objectTracker[objCount].type = t;

                 if (objectTracker[objCount].left +1 >= objectTracker[objCount].right || checkMerge(objectTracker, objectTracker[objCount], objCount)>=0)
                 {
                    objectTracker[objCount].left = 300; // rightmost val
                    objectTracker[objCount].right = -1; // leftmost val
                    objectTracker[objCount].bottom = 0; // topmost val

                 }
                 else
                 objCount++;

              }

            }
        }
    }

    CameraResult result = { .objs_ptr = objectTracker, .len = objCount };

    return result;
}

double centering_coef1(Coords obj, int h){

    return (1 - 0.5*fabs((obj.left + obj.right) / 2.0 - h) / h);
}

double centering_coef4(Coords obj, int h){

    return  pow(1 - 1.0*fabs((obj.left + obj.right) / 2.0 - h) / h, 4);
}

double reverse_centering_coef(Coords obj, int h){

    return 0.5*fabs((obj.left + obj.right) / 2.0 - h) / h;
}

// value of an object in a image with height h
long value(Coords obj, int h){
  if (obj.bottom < h/2-4){
    return 0;
  }
  long x= pow(obj.bottom-h/2 + 4, 4);
  printf("for object l: %d, r: %d, b: %d, value is: %ld\n", obj.left, obj.right, obj.bottom, x);
  return x;
}

// calculate image value through linear combination of each feature (zombies, berries, walls ,etc.)
double getImageValue(CameraResult result, int imageHeight){
  Coords * objs_ptr = result.objs_ptr;
  double sum = 0;
  for (int i = 0; i < result.len; i++) {
    TYPE type = objs_ptr[i].type;
    printf("sum is: %f\n", sum);

    // c contains weight for each object type (see above)
    if (type < 4) {
      // avoid zombies in the middle
      int update = c[type] * value(objs_ptr[i], imageHeight) * centering_coef1(objs_ptr[i], imageHeight);
      sum += update;
    }
    else {
      // priotize berries in the middle
      int update = c[type] * value(objs_ptr[i], imageHeight) * centering_coef4(objs_ptr[i], imageHeight);
      if (type == Wall) {
        // printf(" type=%d, right=%d, left=%d, bottom=%d, ctype=%f, val=%ld, cf=%f, update=%d \n",  type, right, left, bottom, c[type], value(objs_ptr[i], imageHeight), centering_coef4(objs_ptr[i], imageHeight), update);
      } else if (type == 8) {
        // printf("!! type=%d, right=%d, left=%d, bottom=%d, ctype=%f, val=%ld, cf=%f, update=%d \n",  type, right, left, bottom, c[type], value(objs_ptr[i], imageHeight), centering_coef4(objs_ptr[i], imageHeight), update);

      }
      sum += update;
    }
  }

  return sum;
}

// determine if certain local conditions are met and return corresponding direction
int local_control(CameraResult r[4], int imageHeight, bool losing_health){
  // 4 pairs of {zombie type, zombie bottom} for each dir
  int hasZombie[4][2] = {{-1}};
  bool hasZombieFlag = false; // true if there's zombie in the front or back, or if there's a wall
  int threshold = 35;


  for (int x=0;x<4;x++) {
    CameraResult result = r[x];
    // keep track of the closest zombie
    int closest_type = -1;
    int closest_bottom = -1;
     // check really close zombie (likely chased) in each dir
    for (int i = 0; i < result.len; i++) {
      int bottom = result.objs_ptr[i].bottom;
      TYPE type = result.objs_ptr[i].type;
      if (type < 4 && bottom >= threshold) {
      //   printf("aquazombie in dir %d!\n", x);
        closest_type = type;
        closest_bottom = bottom;
        hasZombieFlag = true;
      } else if ((type == Wall || type == BoundaryWall) && bottom >= 45) {
        printf("wall in dir %d!\n", x);
        hasZombieFlag = true;
        closest_bottom = bottom;
        closest_type = type;
      }

    }
    hasZombie[x][0] = closest_type;
    hasZombie[x][1] = closest_bottom;
  }
  // Note: 0,1,2,3 => left, front, right, back

  if (hasZombieFlag || losing_health){
      if (hasZombie[1][0] > -1 && hasZombie[3][0] == -1) {
        // go backward
        return 3;
      } else if (hasZombie[1][0] > -1 && hasZombie[3][0] > -1) {
        // turn to left or right, whichever doesn't have a zombie
        // turn left is faster (so make it turn right, which in reality turns left)
        return hasZombie[0][0] == -1 ? 0 : 2;
      } else {
        // default: go forward
        return 1;
      }
  }

  // check middle berry
    int c[4] = {1,3,0,2}; // change order of dir to be searched: prioritize front and back
    for (int i = 0; i < 4; i++) {
      int x = c[i]; // curr dir
      CameraResult result = r[x];
      for (int i = 0; i < result.len; i++) {
        int left = result.objs_ptr[i].left;
        int right = result.objs_ptr[i].right;
        int bottom = result.objs_ptr[i].bottom;
        TYPE type = result.objs_ptr[i].type;
        if (type >= 4 && type <=7) {
          // it's a central close berry
          if (bottom >= threshold && (left <= imageHeight && right >= imageHeight  ))  {
            printf("find middle berry in direction %d, type: %d\n", x, type);
            return x;
          }
        }
      }
    }
    return -1; // nothing interesting
}

robotControlResult robot_control(bool can_turn, int berry_table[4][4], int* will_be_eaten, bool losing_health)
{
  CameraResult r[4];
  long v[4];
  int best_dir = 0, worst_dir = 0;

  // left
  const unsigned char* image_left = wb_camera_get_image(10);
  r[0] = parse_camera(image_left, 128);

  // right
  const unsigned char* image_right = wb_camera_get_image(9);
   r[2] = parse_camera(image_right, 128);

  // front
  const unsigned char* image_front = wb_camera_get_image(4);
  r[1] = parse_camera(image_front, 128);

  // back
  const unsigned char* image_back = wb_camera_get_image(8);
  r[3] = parse_camera(image_back, 128);

  bool nearStump = false;
  // for each direction, get value for each image
  for (int x=0;x<4;x++){
    CameraResult result = r[x];
    // check front and back camera for will-be-eaten berry and stump
    if (x == 1 || x == 3) {
      printf("checking camera %d result for berry!\n", x);
      for (int i = 0; i < result.len; i++) {
        int bottom = result.objs_ptr[i].bottom;
        TYPE type = result.objs_ptr[i].type;
        // deal with stump
        if (bottom == 63 && type == Stump) {
          nearStump = true;
        }
        if (bottom == 63 && type >= 4 && type <=7) {
          printf("will be eating berry of type %d!\n", type);
          // remember that this berry will be eaten
          *will_be_eaten = type;
          break; // assume only one will be eaten soon
        }
      }
    }
    // remember the best and worst direction
    v[x] = getImageValue(r[x], 64);
    v[x] +=  c_curDir * (x==1);
    if (v[x]>v[best_dir]) best_dir = x;
    if (v[x]<v[worst_dir]) worst_dir = x;
  }

  robotControlResult res = { .global_best_dir = best_dir, .local_best_dir = local_control(r, 64, losing_health), .nearStump = nearStump};
  //printf("%d, %d\n", res.global_best_dir, res.local_best_dir);
  for (int x=0;x<4;x++){
    free(r[x].objs_ptr);
  }
  return res;
}

// called when a berry type has updates in the table; readjust weight if necessary
void update_berry_weight(int berry_table[4][4], struct Robot robot_info, int type) {

  int col = type - 4; // convert 0,1,2,3 to 4,5,6,7

  int add_health = berry_table[0][col];
  int add_energy = berry_table[1][col];
  int lose_energy = berry_table[2][col];
  int health = robot_info.health;
  int energy = robot_info.energy;

  double new_w;

  if (health < 80 && add_health > 0 && add_health > lose_energy) {
    new_w = 80/health;
    // printf("updating berry %d weight to %f", type, new_w);
    c[type] = new_w;

  } else if (health < 80 && add_health > 0 && add_health < lose_energy) {
    new_w = 80/health - 1;
    // printf("updating berry %d weight to %f", type, new_w);
    c[type] = new_w;

  } else if (energy < 60 && add_energy > 0 && add_energy > lose_energy) {
    new_w = 60/energy;
    // printf("updating berry %d weight to %f", type, new_w);
    c[type] = new_w;

  } else if (energy < 60 && add_energy < lose_energy) {
    new_w = 0;
    // printf("updating berry %d weight to %f", type, new_w);
    c[type] = new_w;

  } else if (add_health > lose_energy || add_energy > lose_energy) {
    new_w = 1.25;
    // printf("updating berry %d weight to %f", type, new_w);
    c[type] = new_w;
  }
}

void push_berry() {
  // keep going forward for 6s so robot's very close to stump
  go_forward();
  passive_wait(6.0);
  arm_set_sub_arm_rotation(ARM1, 0);
  arm_set_sub_arm_rotation(ARM2, -0.35);
  arm_set_sub_arm_rotation(ARM5, 0);

   // push berry to the robot's side
   for (double rad = 0.; rad >= -2; rad = rad - 0.2) {
      printf("moving arm 3 to radian %f ********\n", rad);
      arm_set_sub_arm_rotation(ARM3, rad);
      passive_wait(0.1);
      for (double rad = 0.1; rad >= -1.2; rad = rad - 0.2) {
        printf("moving arm 4 to radian %f ********\n", rad);
        arm_set_sub_arm_rotation(ARM4, rad);
        passive_wait(0.1);
      }
   }
   // try moving forward and backward in case berry is stuck
  go_forward();
  passive_wait(2.0);
  go_backward();
  passive_wait(2.0);
  go_forward();
  passive_wait(2.0);

  // push sideaways in case berry is not pushed
  arm_set_sub_arm_rotation(ARM3, -1.5);
  arm_set_sub_arm_rotation(ARM4, -0.89);
  for (double rad = -1; rad <= 1; rad = rad + 0.25) {
    printf("moving arm 1 to radian %f ********\n", rad);
    arm_set_sub_arm_rotation(ARM1, rad);
    passive_wait(0.3);
 }
 // try pushing sideaways with a wider reach
 arm_set_sub_arm_rotation(ARM3, -1.2);
 arm_set_sub_arm_rotation(ARM4, -0.89);
 for (double rad = -1; rad <= 1; rad = rad + 0.25) {
    printf("moving arm 1 to radian %f ********\n", rad);
    arm_set_sub_arm_rotation(ARM1, rad);
    passive_wait(0.3);

 }
 go_backward();
 passive_wait(2.0);

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{

  struct Robot robot_info = {100,100};
  wb_robot_init();
  base_init();
  arm_init();
  gripper_init();
  passive_wait(0.1);

  //display_helper_message();

  int pc = 0;
  wb_keyboard_enable(TIME_STEP);
  int timer = 0;

  WbNodeRef robot_node = wb_supervisor_node_get_from_def("Youbot");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");

  get_all_berry_pos();

  int robot_not_dead = 1;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  // wb_accelerometer_enable(1,1);
  // wb_gps_enable(2,TIME_STEP);
  // wb_compass_enable(3,TIME_STEP);
  wb_camera_enable(4,TIME_STEP);
  //wb_camera_enable(5,TIME_STEP);
  //wb_camera_enable(6,TIME_STEP);
  // wb_camera_enable(7,TIME_STEP);
  wb_camera_enable(8,TIME_STEP);
  wb_camera_enable(9,TIME_STEP);
  wb_camera_enable(10,TIME_STEP);
  // wb_camera_enable(11,TIME_STEP);
  // wb_gyro_enable(12,TIME_STEP);
  // wb_light_sensor_enable(13,TIME_STEP);
  // wb_receiver_enable(14,TIME_STEP);
  // wb_range_finder_enable(15,TIME_STEP);
  // wb_lidar_enable(16,1);

  // WbDeviceTag lidar = wb_robot_get_device("lidar");
  // wb_lidar_enable_point_cloud(lidar);

  // WbDeviceTag rec = wb_robot_get_device("receiver");

  int i = -1, winner = -1;
  int period = 1, turn_cooldown = -1; // for preventing too much turning
  int direction_vote[] = {0, 0, 0, 0}; // for recording how much
  // remember berry correlation
  int berry_table[4][4] = {0};
  int will_be_eaten = -1; // remember the berry that'll be eaten
  int curr_health = robot_info.health;
  int curr_energy = robot_info.energy;
  int curr_armour = robot_info.armour;
  int past_health;
  int past_energy;
  int past_armour;
  int time_counter = 0; // remember time elapsed since detecting will_be_eaten berry
  // since sometimes robot can't detect a zombie when it steps on the robot, checking health helps us detect that
  bool losing_health=false;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  while (robot_not_dead == 1)
  {

  if (robot_info.health < 0)
    {
    robot_not_dead = 0;
    printf("ROBOT IS OUT OF HEALTH\n");
  }

  if (timer % 2 == 0)
  {
    const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
    check_berry_collision(&robot_info, trans[0], trans[2]);
    check_zombie_collision(&robot_info, trans[0], trans[2]);
    //printf("%f\n", trans[0]);
  }
    if (timer == 16)
    {
        update_robot(&robot_info);
        timer = 0;
    }

    step();



    int c = keyboard(pc);
    pc = c;
    timer=timer+1;


    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // this is called everytime step.
    if (i>=0){
      i++;
      if (i==100){ // changed from 150 to 100 to reduce amount of wait time after turning
        turn_cooldown = 0;
        i = -1;
        period = 0;
      }
      continue;
    }

    ///////////////////////// berry correlation calculation /////////////////////
    if (will_be_eaten != -1) {
      // printf("will be eating berry of type: %d\n", will_be_eaten);

      // record the amount of time elapsed since having seen the will_be_eaten berry
      time_counter++;
      // printf("time elapsed since detecting will_be_eaten berry: %d\n", time_counter);

    }
    // remember the past info
    past_health = curr_health;
    past_energy = curr_energy;
    past_armour = curr_armour; 
    // update w/ curr info
    curr_health = robot_info.health; 
    curr_energy = robot_info.energy;
    curr_armour = robot_info.armour;
    
    // check if robot is losing health right now
    if (curr_health < past_health) {
      // printf("losing health!! Is attacked by zombie");
      losing_health = true;
    } else {
      losing_health = false;
    }

    bool updated = false; // record if we made a berry table update
    if (will_be_eaten != -1) {
      int will_be_eaten_col = will_be_eaten - 4; // convert 4,5,6,7 to 0,1,2,3
      printf("updating berry col: %d\n", will_be_eaten_col);
      if (past_health < curr_health || (time_counter > 100 && past_health == 100)) {
        // eaten a berry that boosts 20 health
        berry_table[0][will_be_eaten_col]++;
        // printf("updating berry table for effect 0\n");
        updated = true;
      } else if (past_energy < curr_energy) {
        // eaten a berry that boosts 40 energy
        berry_table[1][will_be_eaten_col]++;
        // printf("updating berry table for effect 1\n");
        updated = true;
      } else if (past_energy - curr_energy >= 20) {
        // eaten a berry that costs 20 energy
        berry_table[2][will_be_eaten_col]++;
        // printf("updating berry table for effect 2\n");
        updated = true;
      } else if (past_armour < curr_armour) {
        // eaten a berry that gives an armour
        berry_table[3][will_be_eaten_col]++;
        // printf("updating berry table for effect 3\n");
        updated = true;
      }
    }

    // reset variables if we're done updating the berry table
    if (updated) {
      // update berry weight
      update_berry_weight(berry_table, robot_info, will_be_eaten);

      // reset time counter
      time_counter = 0;

      printf("with update: check berry table\n");
      for (int i = 0; i < 4; i++) {
          printf("*****row %d******\n", i);
          for (int j = 0; j < 4; j++) {
              printf("  %d  ", berry_table[i][j]);
          }
          printf("\n");
      }
      // reset will_be_eaten
      will_be_eaten = -1;
      // printf("resettig time_counter and will_be_eaten: %d, %d\n", time_counter, will_be_eaten);
    }

    ////////////////////////// get new output from robot_control //////////////
    robotControlResult robot_control_result = robot_control(turn_cooldown <0, berry_table, &will_be_eaten, losing_health);
    int rc = robot_control_result.global_best_dir;
    int local_dir = robot_control_result.local_best_dir;
    // printf("local best direction is: %d\n", local_dir);
    
    ///////////////////////// dealing with stump //////////////////////////////
    bool nearStump = robot_control_result.nearStump;
    // only get berries from stump when robot is healthy and there's no local direction
    if (nearStump && curr_health > 80 && curr_energy > 80 && local_dir == -1) {
      push_berry();
    }
    
    ///////////////////////// voting /////////////////////////////////////////
    direction_vote[rc]++;
    period++;
    // prevent turning with a cool down period
    if (turn_cooldown >=0){
        turn_cooldown++;
        if (turn_cooldown == 150) turn_cooldown = -1;
    }
    // to avoid flipping back and forth between two directions with similar values
    // get accumulated vote based on a period (set to 32 after tuning)
    if (period % 32 == 0 || local_dir != -1){
      period = 0;
      //find winner based on votes
      winner = 1;
      for (int x=0;x<4;x++){
        // printf("dir: %d, votes: %d\n",x,  direction_vote[x]);
        if (direction_vote[x]>direction_vote[winner]) winner = x;
      }
      for (int x=0;x<4;x++) direction_vote[x] = 0;

      if (local_dir!=-1) i = - local_dir % 2;
      else i = -winner % 2;
    }
    // let local best direction overwrites global best direction if needed
    int final_dir = local_dir != -1 ? local_dir : winner;

    // based on the final direction, make robot take the corresponding move
    switch(final_dir) {
      case 1:
        // printf("going forward\n");
        go_forward();
        break;

      case 2:
        stop();
        // printf("turning right\n");
        turn_left();
        break;

      case 0:
        stop();
        // printf("turning left\n");
        turn_right();
        break;
      case 3:
        // printf("going backward\n");
        go_backward();
        break;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
  }

  wb_robot_cleanup();

  return 0;
}
