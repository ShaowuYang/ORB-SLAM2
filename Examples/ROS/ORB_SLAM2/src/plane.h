#ifndef PLANE_H
#define PLANE_H
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/// a class for plane data and methods
/// different plane might be detected and maintained
class Plane
{
public:
  Plane(){ init(); }
  Plane(const Eigen::Vector3d& n);
  Plane(double nx, double ny, double nz);

  void init();

  // Set n and d from input. Keep plane's stats
  bool setPlaneFromPoint(const Eigen::Vector3d& n,
                         const Eigen::Vector3d& p);

  bool createFromPoints(const Eigen::Vector3d& a,
                        const Eigen::Vector3d& b,
                        const Eigen::Vector3d& c);

  inline double diff(const Eigen::Vector3d& p) const {
      return n.dot(p) - d;
  }

  inline bool isInlier(const Eigen::Vector3d& p, double threshold) const {
      if (!valid)
          return false;
      return (fabs(diff(p)) < threshold);
  }

  inline bool isBelow(const Eigen::Vector3d& p) const {
      return (diff(p) > 0);
  }

  inline static bool compare(const Plane& h1, const Plane& h2) {
      return h1.score > h2.score;
  }

  inline static bool comparePoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
      return p1[2] < p2[2];
  }

  inline Sophus::SE3d camTground () const {
      Eigen::Vector3d o = n*d; // ground origin in camera frame
      return Sophus::SE3d(camRground(), o);
  }

  inline Sophus::SE3d groundTcam() const {
      return camTground().inverse();
  }

  inline Eigen::Matrix3d groundRcam() const {
      return camRground().transpose();
  }

  inline Eigen::Matrix3d camRground() const {
      // Change of basis: R is ^camera R_ground (ground to camera).
      Eigen::Matrix3d R;
      Eigen::Vector3d zGround = -n; // n points downward since d > 0
      Eigen::Vector3d yGround = zGround.cross(Eigen::Vector3d(0, 0, 1.));
      yGround /= yGround.norm();
      Eigen::Vector3d xGround = yGround.cross(zGround);

      R.col(0) = xGround;
      R.col(1) = yGround;
      R.col(2) = zGround;

      return R;
  }

  int markInliers(const std::vector<Eigen::Vector3d>& points, std::vector<bool>& inlier, double inlierThreshold);

  Eigen::Vector3d n; // points X on plane satisfy: n*X - d = 0 with d > 0
  double d;
  double score;
  bool valid;
  bool refined;

  int nPoints;
  int nInliers;

  Eigen::Vector3d mean_;
};

class PlaneFinder
{
public:
  PlaneFinder();
  Plane detectPlanesFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);
  Plane findPlane(const std::vector<Eigen::Vector3d>& points, const std::vector<bool>& inlier);
  Plane refinePlane(const std::vector<Eigen::Vector3d>& points, const std::vector<bool>& inlier);

private:
  inline unsigned int preemption(unsigned int i) {
      return nHyp_* std::pow(2.0, - (int) (i/blockSize_));
  }
  void extractInliers(const std::vector<Eigen::Vector3d>& points, const std::vector<bool>& inlier,
                      std::vector<Eigen::Vector3d>& pointsOut);
  void computeMeanAndCovariance(const std::vector<Eigen::Vector3d>& points,
                                const std::vector<double>& weights,
                                Eigen::Vector3d& mean,
                                Eigen::Matrix3d& cov);
  void computeWeights(const std::vector<Eigen::Vector3d>& points,
                      const Eigen::Vector3d& mean,
                      const Eigen::Matrix3d& cov,
                      std::vector<double>& weights);
  inline double weight(double d) {
    b1_ = 2.0; b2_ = 1.25;
      static double d0 = std::sqrt(3.0) + b1_/sqrt(2.0);

      double omega;

      if (d <= d0) {
          omega = d;
      } else {
          double a = (d - d0)/b2_;
          omega = d0*exp(-0.5*a*a);
      }

      return omega/d;
  }

  Plane lastPlane_;
  Plane bestFit_;
  double belowFactor_;
  double b1_, b2_;
  bool  gotInitPlane;

  std::vector<Plane> hypotheses_;
  double angularTolerance_;
  double hightTolerance_;
  unsigned int minInliers_;
  unsigned int nHyp_;
  unsigned int blockSize_;
  unsigned int nObs_;
  int nInliers_;
  int nLowPoints; // expect how many points on the ground have low Z value

  double inlierThreshold_;

};

#endif // PLANE_H
