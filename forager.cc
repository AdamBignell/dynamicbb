#include "stage.hh"
#include <iostream>
#include "canvas.hh"
#include "worldfile.hh"
using namespace Stg;

static const double cruisespeed = 0.4;
static const double avoidspeed = 0.05;
static const double avoidturn = 0.6;
static const double homingTurn = 0.4;
static const double obstructionturn = 0.75; // How drastic is our turn when physically obstructed?
static const double boundaryTurn = 0.5; // How drastic if our turn when outOfBounds?
static const double minfrontdistance = 1.0;
static const bool verbose = false;
static const double stopdist = 0.5;
static const int avoidduration = 10;
static const double defaultRadius = 2.7;
static const double pickupRange = 0.5; // How close a robot needs to be to pick up a puck
static const double seePuckRange = 2.0; // For counting pucks in dynamic relocation
static double globalDistance = 0;
static bool printedDistance = false;
static int numBots = 0;
// This determines when certain values are printed. It does not control simtime itself
static double simTime = 1800; // in seconds = 30 minutes

static const bool dynamicRelocation = false;
 // Relocate must be between 0 and 1
 // This is the 'amount of the way' we move towards where we found a puck
static const bool relocate = true;
static const double relocateWeight = 1.0;

typedef enum { MODE_SEEKING = 0, MODE_HOLDING, MODE_RETURNING} forage_mode_t;

typedef struct {
  ModelPosition *pos;
  ModelRanger *laser;
  ModelFiducial *fiducial;
  ModelFiducial::Fiducial *closest;
  Model *holding;
  double workRadius;
  double p; // Relocate parameter
  Model *sink;
  int avoidcount, randcount;
  Pose workCenter;
  forage_mode_t mode;
  double turnVector;
  double totalDistance;
  Pose prevPose;
} robot_t;

int LaserUpdate(Model *mod, robot_t *robot);
int PositionUpdate(Model *mod, robot_t *robot);
int FiducialUpdate(ModelFiducial *fid, robot_t *robot);
double computeEuclidDistance(const Pose& p0, const Pose& p1);
double computeBearing(const Pose& from, const Pose& to);
void troubleShootStalling(robot_t *robot, bool obstruction, bool stop, bool outOfBounds);
void turnToSink(robot_t *robot, bool outOfBounds);
void turnToWork(robot_t *robot, double distance);
//void initializePucks(Model *mod, robot_t *robot);

// Stage calls this when the model starts up
extern "C" int Init(Model *mod, CtrlArgs *)
{
  numBots++;
  // local arguments
  /*  printf( "\nWander controller initialised with:\n"
      "\tworldfile string \"%s\"\n"
      "\tcmdline string \"%s\"",
      args->worldfile.c_str(),
      args->cmdline.c_str() );
  */
  robot_t *robot = new robot_t();

  robot->totalDistance = 0;
  robot->avoidcount = 0;
  robot->randcount = 0;
  robot->turnVector = 0;
  robot->mode = MODE_SEEKING;

  robot->pos = dynamic_cast<ModelPosition *>(mod);
  if (!robot->pos) {
    PRINT_ERR("No position model given in wander controller.");
    exit(1);
  }

  robot->pos->AddCallback(Model::CB_UPDATE, model_callback_t(PositionUpdate), robot);
  robot->pos->Subscribe(); // starts the position updates

// Start the fiducial updates
  robot->fiducial = (ModelFiducial *)mod->GetUnusedModelOfType("fiducial");
  robot->fiducial->AddCallback(Model::CB_UPDATE, (model_callback_t)FiducialUpdate, robot);
  robot->fiducial->Subscribe();

  // New additions here for foraging
  robot->workRadius = defaultRadius;
  robot->workCenter = robot->pos->GetPose();
  robot->prevPose = robot->pos->GetPose();
  // printf("\n\nWorkCenter X = %f", robot->workCenter.x);
  // printf("\nWorkCenter Y = %f", robot->workCenter.y);

  // Initialize the sink
  robot->sink = mod->GetWorld()->GetModel("sink");

  // find a range finder

  ModelRanger *laser = NULL;

  // printf( "\nWander ctrl for robot %s:\n",  robot->pos->Token() );
  for( int i=0; i<16; i++ )
    {
      char name[32];
      snprintf( name, 32, "ranger:%d", i ); // generate sequence of model names
      //
      // printf( "  looking for a suitable ranger at \"%s:%s\" ... ", robot->pos->Token(), name );
      laser = dynamic_cast<ModelRanger *>(robot->pos->GetChild( name ));

      if( laser && laser->GetSensors()[0].sample_count > 8 )
	{
	  // puts( "yes." );
	  break;
	}

      // puts( "no." );
    }

  if( !laser ) {
    PRINT_ERR("  Failed to find a ranger with more than 8 samples. Exit.");
    exit(2);
  }

  robot->laser = laser;
  robot->laser->AddCallback(Model::CB_UPDATE, model_callback_t(LaserUpdate), robot);
  robot->laser->Subscribe(); // starts the ranger updates

  // if (!pucksInit)
  // {
  //   initializePucks(robot);
  //   pucksInit = true;
  // }

  return 0; // ok
}


void seeking(Model *mod, robot_t *robot)
{
    // This method does the job laserUpdate used to do in wander
    // get the data
    const std::vector<meters_t> &scan = robot->laser->GetSensors()[0].ranges;
    uint32_t sample_count = scan.size();
    if (sample_count < 1)
      return;

    bool obstruction = false;
    bool stop = false;
    bool outOfBounds = false;

    // find the closest distance to the left and right and check if
    // there's anything in front
    double minleft = 1e6;
    double minright = 1e6;

    for (uint32_t i = 0; i < sample_count; i++) {
      if (verbose)
        printf("%.3f ", scan[i]);

      if ((i > (sample_count / 3)) && (i < (sample_count - (sample_count / 3)))
          && scan[i] < minfrontdistance) {
        if (verbose)
          puts("  obstruction!");
        obstruction = true;
      }

      if (scan[i] < stopdist) {
        if (verbose)
          puts("  stopping!");
        stop = true;
      }

      if (i > sample_count / 2)
        minleft = std::min(minleft, scan[i]);
      else
        minright = std::min(minright, scan[i]);
    }

    // Check if the robot is outside the working Radius
    // This is a convenient place to do this since we are already steering here
    double distance = computeEuclidDistance(robot->pos->GetPose(), robot->workCenter);
    if(distance > robot->workRadius && robot->mode != MODE_RETURNING)
    {
      outOfBounds = true;
    }

    if (verbose) {
      puts("");
      printf("minleft %.3f \n", minleft);
      printf("minright %.3f\n ", minright);
    }



    if (obstruction || stop || (robot->avoidcount > 0)) {
      if (verbose)
        printf("Avoid %d\n", robot->avoidcount);

      robot->pos->SetXSpeed(stop ? 0.0 : avoidspeed);

      /* once we start avoiding, select a turn direction and stick
   with it for a few iterations */
      if (robot->avoidcount < 1) {
        robot->turnVector = 0;
        if (verbose)
          puts("Avoid START");
        robot->avoidcount = random() % avoidduration + avoidduration;

        if (obstruction)
          {
          if (minleft < minright) {
            robot->turnVector -= obstructionturn;
            if (verbose)
              printf("turning right %.2f\n", -obstructionturn);
          } else {
            robot->turnVector += obstructionturn;
            if (verbose)
              printf("turning left %2f\n", +obstructionturn);
          }
        }
      }

      robot->avoidcount--;
    } else {
      if (verbose)
        puts("Cruise");

      robot->avoidcount = 0;
      robot->pos->SetXSpeed(cruisespeed);
      robot->turnVector = 0;
    }

    if (robot->avoidcount < 1)
    {
      // Below are special cases
      // If we are holding, we home towards the sink
      // Note this will stack with obstacles
      if (robot->mode == MODE_HOLDING)
      {
        turnToSink(robot, outOfBounds);
      }
      // If we are returning to the workarea we need to seek the center
      else if (robot->mode == MODE_RETURNING || outOfBounds)
      {
        turnToWork(robot, distance);
      }
      troubleShootStalling(robot, obstruction, stop, outOfBounds);
    }

    robot->pos->SetTurnSpeed(robot->turnVector);
    return; // run again
}

void homing(Model * mod, robot_t *robot)
{
  seeking(mod, robot);
}

void returning(Model * mod, robot_t *robot)
{
  // Returning is just seeking with bias towards the work area
  seeking(mod, robot);
}

// Head to the drop-off location (sink)
void turnToSink(robot_t *robot, bool outOfBounds)
{
  double goHome = homingTurn * computeBearing(robot->pos->GetPose(), robot->sink->GetPose());
  robot->turnVector += goHome;
  // If we are out of bounds we need to get back home
  if (computeEuclidDistance(robot->pos->GetPose(), robot->sink->GetPose()) < 1 ||
      outOfBounds)
  {
    robot->mode = MODE_RETURNING;
    Pose puckPose = robot->pos->GetPose();
    puckPose.z = 0.2;
    (robot->holding)->SetPose(puckPose);
    robot->holding = NULL;
  }
}

// Head back towards our working area
void turnToWork(robot_t *robot, double distance)
{
  double backToBounds = 0.5 * computeBearing(robot->pos->GetPose(), robot->workCenter);
  robot->turnVector += backToBounds;
  // If the distance to home is sufficiently close
  if (distance < defaultRadius/2)
  {
    robot->mode = MODE_SEEKING;
  }
}

void troubleShootStalling(robot_t *robot, bool obstruction, bool stop, bool outOfBounds)
{
  // Various control strategies could work here
  if( robot->pos->Stalled())
  {
      robot->pos->SetXSpeed(-0.01);
  }
  // Wiggle out of stasis
  if ((obstruction || stop || outOfBounds) && (robot->avoidcount < 1 && robot->turnVector < 0.1))
  {
    robot->turnVector = 0.5 + ((double)rand() / (double)RAND_MAX);
    double coinFlip = (double)rand() / (double)RAND_MAX;
    if (coinFlip >= 0.5)
    {
      robot->turnVector *= -1;
    }
    robot->avoidcount = random() % avoidduration;
    robot->pos->SetXSpeed(random());
  }
}

// inspect the ranger data and decide what to do
int LaserUpdate(Model *mod, robot_t *robot)
{
  if (robot->holding)
  {
    Pose puckPose = robot->pos->GetPose();
    puckPose.z = 0.5;
    (robot->holding)->SetPose(puckPose);
  }
  switch (robot->mode)
  {
    case MODE_SEEKING:
      seeking(mod, robot);
      break;

    case MODE_HOLDING:
      homing(mod, robot);
      break;

    case MODE_RETURNING:
      returning(mod, robot);
      break;

    return 0;
  }
  return 0;
}

int PositionUpdate(Model *mod, robot_t *robot)
{
  // Just do this hear since this is called frequenctly
  Pose pose = robot->pos->GetPose();
  double distanceTravelled = computeEuclidDistance(pose, robot->prevPose);
  robot->totalDistance += distanceTravelled;
  globalDistance += distanceTravelled;
  robot->prevPose = pose;
  double seconds = mod->GetWorld()->SimTimeNow() / 1e6;
  if (!printedDistance && seconds >= (simTime - 0.1))
  {
    printf("Global Distance = [%.5f meters]\n",  globalDistance);
    printf("Avg Distance = [%.5f meters]\n",  globalDistance/numBots);
    printedDistance = true;
  }
  //printf("Pose: [%.2f %.2f %.2f %.2f]\n", pose.x, pose.y, pose.z, pose.a);
  // Crumb attempt
  //allPucks->drawAllCrumbs(mod, color, robot->pos);

  return 0; // run again
}

double computeEuclidDistance(const Stg::Pose& p0, const Stg::Pose& p1)
{
  const double x_diff = p0.x - p1.x;
  const double y_diff = p0.y - p1.y;
  return sqrt(x_diff * x_diff + y_diff * y_diff);
}

double computeBearing(const Stg::Pose& from, const Stg::Pose& to)
{
   return normalize(atan2(to.y - from.y, to.x - from.x) - from.a);
}

int FiducialUpdate(ModelFiducial *fid, robot_t *robot)
{
  // Get fiducials
  double dist = 1e6; // big


  if (robot->mode == MODE_SEEKING)
  {
    int totalSeen = 0;
    FOR_EACH (it, fid->GetFiducials()) {
      ModelFiducial::Fiducial *other = &(*it);
      if (other->range < seePuckRange)
      {
        totalSeen++;
      }
      // Ensure the puck is close enough to pick up
      // and we only pick up pucks
      if (other->id == 10 && other->range < pickupRange) {
            robot->holding = other->mod;
            robot->mode = MODE_HOLDING;
            if (!dynamicRelocation && relocate)
            {
              Pose pose = robot->pos->GetPose();
              double xDist = pose.x - robot->workCenter.x;
              double yDist = pose.y - robot->workCenter.y;
              robot->workCenter.x = robot->workCenter.x + relocateWeight*xDist;
              robot->workCenter.y = robot->workCenter.y + relocateWeight*yDist;
              if (verbose)
              {
                std::cout << "New Working Area: " << robot->workCenter.x << ", " << robot->workCenter.y << '\n';
              }
            }
        }
    }

    // The heart of the dynamic control algorithm is here
    if (dynamicRelocation && robot->holding)
    {
      Pose pose = robot->pos->GetPose();
      double xDist = pose.x - robot->workCenter.x;
      double yDist = pose.y - robot->workCenter.y;
      double dynamicWeight = (totalSeen/10.0);
      if (dynamicWeight >= 1.0)
      {
        dynamicWeight = 1.0; // Hard cap on relocation
      }
      robot->workCenter.x = robot->workCenter.x + dynamicWeight*xDist;
      robot->workCenter.y = robot->workCenter.y + dynamicWeight*yDist;
      if (verbose)
      {
        std::cout << "TotalSeen = " << totalSeen << '\n';
        std::cout << "dynamicWeight = " << dynamicWeight << '\n';
        std::cout << "New Working Area: " << robot->workCenter.x << ", " << robot->workCenter.y << '\n';
      }
    }
  }
  return 0;
}
