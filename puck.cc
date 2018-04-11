
#include "stage.hh"
#include <iostream>
#include <math.h>
#include <time.h>
using namespace Stg;

static int throughput = 0;
static bool randomPlacement = true;
const double size = 8;
static int totalPucks = 0;
static const int numClusters = 1;
static double clusterX[numClusters];
static double clusterY[numClusters];
static bool first = true;
static bool clustered = false;
static double clusterParam = 5;
static const bool embedded = true;
static const double ratioInside = 0.75;
static const int hardCodeNumPucks = 256; //Shouldn't rely on this, totalPucks is better
static int numInside = 0;

typedef struct {
  Model* sink;
  Model* pos;
  bool home;
} puck_t;

double computeEuclidDistance(const Pose& p0, const Pose& p1);
void placePuck(Pose &newPose, bool cluster);

int Update(Model *mod, puck_t *info)
{
  Pose p0 = info->pos->GetPose();
  Pose p1 = info->sink->GetPose();
  if (computeEuclidDistance(p0, p1) < 1 && p0.z != 5 && !info->home)
  {
    throughput++;
    info->home = true;
    if (embedded)
    {
      if ((rand() % 4)/4 <= ratioInside)
      {
        // Place randomly in cluster
        placePuck(p0, true);
      }
      else
      {
        // Place randomly in the world
        placePuck(p0, false);
      }
    }
    else
    {
      placePuck(p0, clustered);
    }
    // p0.z = 5;
    info->pos->SetPose(p0);
    int seconds = mod->GetWorld()->SimTimeNow() / 1e6;
    int minutes = 0;
    if (seconds >= 60)
    {
      minutes = floor(seconds/60);
    }
    int secondsOver = seconds - 60*minutes;
    std::cout << "Total pucks home: " << throughput << " at time: " << minutes << ':' << secondsOver << "\n";
  }
  return 0; // run again
}

// Stage calls this when the model starts up
extern "C" int Init(Model *mod, CtrlArgs *args)
{
  if (first) // do some set-up
  {
    if (clusterParam == 0)
    {
      clustered = false;
    }
    srand(time(NULL));
    for (int i = 0; i < numClusters; ++i)
    {
      // This determines the cluster center
      // and fixes any problems (such as cluster extending beyond the arena edge)
      int isNegativeClusterX = 1;
      int isNegativeClusterY = 1;
      if (rand() % 2 == 0)
      {
        isNegativeClusterX = -1;
      }
      if (rand() % 2 == 0)
      {
        isNegativeClusterY = -1;
      }
      clusterX[i] = isNegativeClusterX * static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(size-0.5)));
      clusterY[i] = isNegativeClusterY * static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(size-0.5)));
      Pose p1 = mod->GetWorld()->GetModel("sink")->GetPose();
      Pose clusterPose;
      clusterPose.x = clusterX[i];
      clusterPose.y = clusterY[i];
      // Initial placement ensures the cluster doesn't overlap the sink
      while (computeEuclidDistance(clusterPose, p1) < (1/clusterParam)*size)
      {
        clusterX[i] = isNegativeClusterX * static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(size-0.5)));
        clusterY[i] = isNegativeClusterY * static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(size-0.5)));
        clusterPose.x = clusterX[i];
        clusterPose.y = clusterY[i];
      }
      std::cout << "Cluster Location X = " << clusterX[i] << '\n';
      std::cout << "Cluster Location X = " << clusterY[i] << '\n';
    }
    first = false;
  }
  puck_t *puck = new puck_t;
  totalPucks++;
  //std::cout << "Initializing Puck #" << totalPucks << '\n';
  puck->pos = mod;
  puck->home = false;
  Pose newPose = puck->pos->GetPose();

  if (embedded)
  {
     if(numInside < hardCodeNumPucks*ratioInside)
     {
       placePuck(newPose, true);
       numInside++;
     }
     else
     {
       placePuck(newPose, false);
     }
  }
  else{
    placePuck(newPose, clustered);
  }

  puck->pos->SetPose(newPose);

  puck->sink = mod->GetWorld()->GetModel("sink");
  assert(puck->sink);

  mod->AddCallback(Model::CB_UPDATE, (model_callback_t)Update, puck);
  mod->Subscribe();
  return 0; // ok
}

double computeEuclidDistance(const Stg::Pose& p0, const Stg::Pose& p1)
{
  const double x_diff = p0.x - p1.x;
  const double y_diff = p0.y - p1.y;
  return sqrt(x_diff * x_diff + y_diff * y_diff);
}

void placePuck(Pose &newPose, bool cluster)
{
  if (!cluster)
  {
    newPose.x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(size-0.5)));
    newPose.y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(size-0.5)));
    if (rand() % 2 == 0)
    {
      newPose.x = newPose.x * -1;
    }
    if (rand() % 2 == 0)
    {
      newPose.y = newPose.y * -1;
    }

    // Sorry for this syntax. Just confirming the values are in range
    if (newPose.x >= size) newPose.x = size - 0.5;
    else if (newPose.x <= (-1)*size) newPose.x =  -1*size + 0.5;

    if (newPose.y >= size) newPose.y = size - 0.5;
    else if (newPose.y <= (-1)*size) newPose.y = -1*size + 0.5;
  }
  if (cluster)
  {
    double a = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
    double b = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX));
    double theta = b * 2 * M_PI;
    double rr = (1/clusterParam)*size * sqrt(a);
    double x = rr * cos(theta);
    double y = rr * sin(theta);
    newPose.x = clusterX[0] + x;
    newPose.y = clusterY[0] + y;
    if ((newPose.x >= size) || (newPose.x <= (-1)*size) || (newPose.y >= size) || (newPose.y <= (-1)*size))
    {
      placePuck(newPose, clustered);
    }
  }
}
