#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l,
                                  Vector3d global_xyz_u, int max_x_id,
                                  int max_y_id, int max_z_id) {//0.2,(-15,-15,0),(15,15,4),150,150,60
  gl_xl = global_xyz_l(0);//-15
  gl_yl = global_xyz_l(1);//-15
  gl_zl = global_xyz_l(2);//0

  gl_xu = global_xyz_u(0);//15
  gl_yu = global_xyz_u(1);//15
  gl_zu = global_xyz_u(2);//4

  GLX_SIZE = max_x_id;//150
  GLY_SIZE = max_y_id;//150
  GLZ_SIZE = max_z_id;//60
  GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;//150*60=9000
  GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;//150*150*60=1350000

  resolution = _resolution;//0.2
  inv_resolution = 1.0 / _resolution;//5.0

  data = new uint8_t[GLXYZ_SIZE];
  memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));

  GridNodeMap = new GridNodePtr **[GLX_SIZE];
  for (int i = 0; i < GLX_SIZE; i++) {
    GridNodeMap[i] = new GridNodePtr *[GLY_SIZE];
    for (int j = 0; j < GLY_SIZE; j++) {
      GridNodeMap[i][j] = new GridNodePtr[GLZ_SIZE];
      for (int k = 0; k < GLZ_SIZE; k++) {
        Vector3i tmpIdx(i, j, k);
        Vector3d pos = gridIndex2coord(tmpIdx);
        GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
      }
    }
  }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr) {
  ptr->id = 0;
  ptr->cameFrom = NULL;
  ptr->gScore = inf;
  ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids() {
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++)
        resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y,
                             const double coord_z) {
  if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
      coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu)
    return;//如果这个点超出边界，则放弃这个点

  int idx_x = static_cast<int>((coord_x - gl_xl) * inv_resolution);//计算出这个点对应的栅格坐标
  int idx_y = static_cast<int>((coord_y - gl_yl) * inv_resolution);
  int idx_z = static_cast<int>((coord_z - gl_zl) * inv_resolution);

  if (idx_x == 0 || idx_y == 0 || idx_z == GLZ_SIZE || idx_x == GLX_SIZE ||
      idx_y == GLY_SIZE)
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
  else {
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y + 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x)*GLYZ_SIZE + (idx_y - 1) * GLZ_SIZE + idx_z] = 1;
    data[(idx_x + 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
    data[(idx_x - 1) * GLYZ_SIZE + (idx_y)*GLZ_SIZE + idx_z] = 1;
  }//将这个点膨胀为一个栅格，即一个点膨胀为９个点
}

vector<Vector3d> AstarPathFinder::getVisitedNodes() {
  vector<Vector3d> visited_nodes;
  for (int i = 0; i < GLX_SIZE; i++)
    for (int j = 0; j < GLY_SIZE; j++)
      for (int k = 0; k < GLZ_SIZE; k++) {
        // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and
        // close list
        if (GridNodeMap[i][j][k]->id ==
            -1) // visualize nodes in close list only
          visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
      }

  ROS_WARN("visited_nodes size : %d", visited_nodes.size());
  return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i &index) {
  Vector3d pt;

  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d &pt) {
  Vector3i idx;
  idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
      min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
      min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d &coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i &index) const {
  return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i &index) const {
  return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int &idx_x, const int &idx_y,
                                        const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int &idx_x, const int &idx_y,
                                    const int &idx_z) const {
  return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE &&
          idx_z >= 0 && idx_z < GLZ_SIZE &&
          (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr,
                                          vector<GridNodePtr> &neighborPtrSets,
                                          vector<double> &edgeCostSets) {
  neighborPtrSets.clear();
  edgeCostSets.clear();
  Vector3i neighborIdx;
  for (int dx = -1; dx < 2; dx++) {
    for (int dy = -1; dy < 2; dy++) {
      for (int dz = -1; dz < 2; dz++) {

        if (dx == 0 && dy == 0 && dz == 0)
          continue;

        neighborIdx(0) = (currentPtr->index)(0) + dx;
        neighborIdx(1) = (currentPtr->index)(1) + dy;
        neighborIdx(2) = (currentPtr->index)(2) + dz;

        if (neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE ||
            neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE ||
            neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE) {
          continue;
        }

        neighborPtrSets.push_back(
            GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)]);
        edgeCostSets.push_back(sqrt(dx * dx + dy * dy + dz * dz));
      }
    }
  }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2) {
  // using digonal distance and one type of tie_breaker.
  double dx,dy,dz,h;
  dx=node1->coord.x()-node2->coord.x();
  dy=node1->coord.y()-node2->coord.y();
  dz=node1->coord.z()-node2->coord.z();
  h=sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2));
  h*=10.0;
  h=h*(1.0f+0.1f);//tie_breaker
  return h;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt) {
  ros::Time time_1 = ros::Time::now();

  // index of start_point and end_point
  Vector3i start_idx = coord2gridIndex(start_pt);
  Vector3i end_idx = coord2gridIndex(end_pt);
  goalIdx = end_idx;

  // position of start_point and end_point
  start_pt = gridIndex2coord(start_idx);
  end_pt = gridIndex2coord(end_idx);

  // Initialize the pointers of struct GridNode which represent start node and
  // goal node
  GridNodePtr startPtr = new GridNode(start_idx, start_pt);
  GridNodePtr endPtr = new GridNode(end_idx, end_pt);

  // openSet is the open_list implemented through multimap in STL library
  openSet.clear();
  // currentPtr represents the node with lowest f(n) in the open_list
  GridNodePtr currentPtr = NULL;
  GridNodePtr neighborPtr = NULL;

  // put start node in open set
  startPtr->gScore = 0;
  /**
   *
   * STEP 1.1:  finish the AstarPathFinder::getHeu
   *
   * **/
  startPtr->fScore = getHeu(startPtr, endPtr);

  startPtr->id = 1;
  startPtr->coord = start_pt;
  openSet.insert(make_pair(startPtr->fScore, startPtr));

  /**
   *
   * STEP 1.2:  some else preparatory works which should be done before while
   * loop
   *
   * **/
  double tentative_gScore;
  vector<GridNodePtr> neighborPtrSets;
  vector<double> edgeCostSets;
  Vector3i XYZ;
  int x,y,z;

  /**
   *
   * STEP 1.3:  finish the loop
   *
   * **/
  while (!openSet.empty()) 
  {
    XYZ=openSet.begin()->second->index;//提取openset中ｆ值最小的点的index
    currentPtr=GridNodeMap[XYZ(0)][XYZ(1)][XYZ(2)];//将ｆ值最小的点赋值为当前点
    GridNodeMap[XYZ(0)][XYZ(1)][XYZ(2)]->id=-1;//将当前点放去到close中
    openSet.erase(openSet.begin());//将当前点从openset删除
    /*
    判断是否到达终点
    */
   if (currentPtr->index==end_idx)
   {
     terminatePtr=currentPtr;
     return;
   }
    AstarGetSucc(currentPtr,neighborPtrSets,edgeCostSets);//通过当前点得到对应的领域点以及对应ｇ值
    for (size_t i = 0; i < neighborPtrSets.size(); i++)
    {
      neighborPtr=neighborPtrSets[i];
      //若是该领域点处于障碍物中，则跳过
      if (isOccupied(neighborPtr->index))
      {
        continue;
      }
      //若是该领域点处于close中，则跳过．
      if (neighborPtr->id==-1)
      {
        continue;
      }
      //若是该领域点已经处于open中，则比较ｇ值，选择是否要替换父节点
      else if (neighborPtr->id==1)
      {
        //如果该该领域点的父节点通过当前点到达领域点的g值小于由该领域点的父节点直接到达领域点的ｇ值，
        //则将该领域点的父节点设置为当前点，并将对应的ｇ,f值修改
        if ((neighborPtr->gScore) > (currentPtr->gScore + edgeCostSets[i]))
        {
          x=neighborPtr->index.x();
          y=neighborPtr->index.y();
          z=neighborPtr->index.z();
          GridNodeMap[x][y][z]->cameFrom=currentPtr;
          GridNodeMap[x][y][z]->gScore=edgeCostSets[i];
          GridNodeMap[X][Y][Z]->fScore=GridNodeMap[x][y][z]->gScore+getHeu(GridNodeMap[x][y][x],endPtr);
        }
      }
      //若是该领域点是一个空白点，即没有加入任何集合，将其加入到openlist中
      else
      {
        neighborPtr->cameFrom=currentPtr;
        neighborPtr->gScore=edgeCostSets[i];
        neighborPtr->fScore=neighborPtr->gScore+getHeu(neighborPtr,endPtr);
        neighborPtr->id=1;
        openSet.insert(make_pair(neighborPtr->fScore,neighborPtr));
      }
    }//所有领域点判断结束
  }//大循环结束
  // if search fails
  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_WARN("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
}

vector<Vector3d> AstarPathFinder::getPath() {
  vector<Vector3d> path;
  vector<GridNodePtr> gridPath;

  /**
   *
   * STEP 1.4:  trace back the found path
   *
   * **/
  while (terminatePtr->cameFrom!=NULL)
  {
    path.push_back(terminatePtr->coord);
    //gridPath.push_back(terminatePtr);
    terminatePtr=terminatePtr->cameFrom;
  }
  reverse(path.begin(),path.end());
  //reverse(gridPath.begin(),gridPath.end());
  return path;
}

vector<Vector3d> AstarPathFinder::pathSimplify(const vector<Vector3d> &path,
                                               double path_resolution) {
  vector<Vector3d> subPath,path1,path2;
  Vector3d b,u,v,p;
  u=path.begin();
  v=path.end();
  double d,d_max=0;
  int index=0;
  
  /**
   *
   * STEP 2.1:  implement the RDP algorithm
   *流程：先找到距离起点与终点连成的直线中，距离最大的点；
   然后将轨迹在距离最大的点处分为两段，继续执行上一步，直到不能再分
   ***/
  //先找到那个距离最大的点
  for (size_t i =1 ; i < path.size()-1; i++)
  {
    ｂ=path[i];
    p＝b-(((v-u)*(v-u).transpose())/((v-u).transpose()*(v-u)))*(b-u);
    d=p.squaredNorm();
    //记录距离最大值点的索引值
    if (d>d_max)
    {
      d_max=d;
      index=i;
    }
  }
  //从距离最大的点处将线段分开，分别进行RDP算法，直到最新线段的两个端点的距离小于阈值
  if (d_max>path_resolution)
  {
    for (size_t j = 0; j <= index; j++)
    {
      path1.push_back(path[j]);
    }
    for (size_t k = index; k <= path.size(); k++)
    {
      path2.push_back(path[k]);
    }
    pathSimplify(path1,path_resolution);
    pathSimplify(path2,path_resolution);
  }
  else
  {
    //subPath.push_back(path.begin());
    subPath.push_back(path.end());
  }
  return subPath;
}

Vector3d AstarPathFinder::getPosPoly(MatrixXd polyCoeff, int k, double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

int AstarPathFinder::safeCheck(MatrixXd polyCoeff, VectorXd time) {
  int unsafe_segment = -1; //-1 -> the whole trajectory is safe
  /**
   *
   * STEP 3.3:  finish the sareCheck()
   *
   * **/

  return unsafe_segment;
}