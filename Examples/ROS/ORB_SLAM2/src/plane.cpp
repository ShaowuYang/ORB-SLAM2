#include <boost/foreach.hpp>
#include <pcl/common/eigen.h>
#include <vector>

#include "plane.h"

Plane::Plane(const Eigen::Vector3d& n)
{
    init();
    double norm = n.norm();
    if (norm != 0.0) {
        d = 1.0/norm;
        this->n = n*d;
        valid = false;
    }
    assert(d >= 0.0);
}

Plane::Plane(double nx, double ny, double nz)
{
    init();
    double norm = std::sqrt(nx*nx + ny*ny + nz*nz);
    if (norm != 0.0) {
        d = 1.0/norm;
        n = Eigen::Vector3d(nx, ny, nz)*d;
        valid = false; /// true
    }

    assert(d >= 0.0);
}

void Plane::init()
{
    score = 0.0;
    valid = false;
    refined = false;

    mean_ = Eigen::Vector3d(0.0, 0.0, 0.0);
}

bool Plane::setPlaneFromPoint(const Eigen::Vector3d& knownN,
                            const Eigen::Vector3d& p)
{
    n = knownN;
    d = n.dot(p);

    if (n[2] < 0.0) {
        d = -d;
        n = -n;
    }

    valid = true;
    assert(!(n[2] < 0.0));

    mean_ = p;
    return true;
}

bool Plane::createFromPoints(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c)
{
    init();

    Eigen::Vector3d p1 = a - b;
    Eigen::Vector3d p2 = a - c;
    Eigen::Vector3d cross = p1.cross(p2);

    double norm = cross.norm();
    if (norm > 1e-5) {
        n = cross / norm;
        d = a.dot(n);

        if (n[2] < 0) {
            d = -d;
            n = -n;
        }
        valid = true;
    }
    assert(n[2] >= 0.0);
    return valid;
}

int Plane::markInliers(const std::vector<Eigen::Vector3d>& points, std::vector<bool>& inlier, double inlierThreshold)
{
    int nInliers = 0;
    inlier.resize(points.size());
    assert(points.size() == inlier.size());
    for (unsigned int i = 0; i < points.size(); i++) {
        inlier[i] = isInlier(points[i], inlierThreshold);
        if (inlier[i])
            nInliers++;
    }
    return nInliers;
}

void randPerm(int n,std::vector<int>& p)
{
    if ((int) p.size() != n)
        p.resize(n);

    for (int i = 0; i < n; i++) {
        int j = rand() % (i + 1);
        p[i] = p[j];
        p[j] = i;
    }
}

void randSampleNoReplacement(int nmax, int nsamples, int* p)
{
   for (int i = 0; i < nsamples; i++) {
        bool unique = false;
        while (!unique)
        {
            p[i] = rand() % nmax;
            unique = true;
            for (int j = 0; j < i; j++)
                if (p[j] == p[i])
                    unique = false;
        }
    }
}

PlaneFinder::PlaneFinder()
{
  // this plane finder will track new plane in the new kf, based on the last plane
  Plane initPlane(Eigen::Vector3d(0., 0., 1.));
  lastPlane_ = initPlane;
  gotInitPlane = false;
  angularTolerance_ = 15.*M_PI/180.;
  hightTolerance_ = 0.2; // hight diff to ground plane in last KF

  minInliers_ = 30;
  nHyp_ = 300;
  blockSize_ = 50;
  nObs_ = 5000;
  nLowPoints = 2000;

  inlierThreshold_ = 0.10;
  belowFactor_ = 1;// 10
}

Plane PlaneFinder::detectPlanesFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
  // STEP 1: Extract Points from pointcloud
  std::vector<Eigen::Vector3d> points;
  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator pc_iter = pointCloud->begin();
      pc_iter!=pointCloud->end();pc_iter++)
  {
    pcl::PointXYZRGB& pt = *pc_iter;
    Eigen::Vector3d point(pt.x, pt.y, pt.z);
    points.push_back(point);
  }
  if (!gotInitPlane){
    nLowPoints = 0.5*points.size();
    std::nth_element(points.begin(),points.begin()+nLowPoints,points.end(),Plane::comparePoints);
    points.resize(nLowPoints);
    gotInitPlane = true;
  }
  std::cout << "Points size: " << points.size() << std::endl;

  // STEP 2: Find inliers / outliers using preemptive RANSAC
  std::vector<bool> inliers;
  Plane plane = findPlane(points, inliers);

  std::cout << "find plane: " << plane.valid << std::endl;

  int nInliersRansac = plane.markInliers(points, inliers, 0.2);
  int nInliersRobust = 0;
  if (nInliersRansac < 0.01*points.size() || nInliersRansac < minInliers_) {
      plane.valid = false;
      nInliers_ = nInliersRansac;
      std::cout << "too few valid points in plane: " << nInliersRansac << std::endl;
  } else {
      plane.valid = true;

      // STEP 3: Refine based on inliners using robust optimization
      plane = refinePlane(points, inliers);
      nInliersRobust = plane.markInliers(points, inliers, inlierThreshold_);

      if (plane.valid)
        plane.valid = (nInliersRobust >= 0.01*points.size() && nInliersRobust >= minInliers_);
      nInliers_ = nInliersRobust;
  }

  std::cout << "refine plane: " << plane.valid << std::endl;

  plane.nInliers = nInliers_;
  plane.nPoints = points.size();
  if (plane.valid)
    lastPlane_ = plane;

  return plane;
}

Plane PlaneFinder::findPlane(const std::vector<Eigen::Vector3d>& points, const std::vector<bool>& inlier)
{
  if (points.size() < minInliers_)
      return Plane();

  std::vector<int> oInd;
  randPerm(points.size(), oInd);
//  std::cout << "lastPlane_ n: " << lastPlane_.n << std::endl;

  // Generate nHyp hypotheses
  hypotheses_.resize(nHyp_);
  for (uint i = 0; i < nHyp_; i++) {
      int currIndices[3];
      hypotheses_[i].valid = false;

      while (!hypotheses_[i].valid) {
          randSampleNoReplacement(points.size(), 3, currIndices);
          hypotheses_[i].createFromPoints(points[currIndices[0]], points[currIndices[1]], points[currIndices[2]]);
      }

      if (angularTolerance_ > 0.0) {
          // Check whether this hypothesis is within range of our initialization constraint.
          double angle = std::acos((double) lastPlane_.n.dot(hypotheses_[i].n));
          assert(angle > 0);
          if (angle > angularTolerance_) {
//            std::cout << "hypotheses_ n: " << hypotheses_[i].n << std::endl;
//            std::cout << "angle diff too large: " << angle << std::endl;
              hypotheses_[i].valid = false;
              hypotheses_[i].score = std::numeric_limits<double>::min();
          }
      }
  }

  if (lastPlane_.valid) {
      // first hypothesis from initialization:
      hypotheses_[0] = lastPlane_;
  }

  // Start preemptive scoring
  unsigned int i = 0;
  unsigned int pr = preemption(i);
  while (i < nObs_ && i < points.size() && pr > 1) {
      // Use observation oInd[i] to evaluate all currently remaining hypotheses:
      const Eigen::Vector3d& o = points[oInd[i]];

      for (std::vector<Plane>::iterator it = hypotheses_.begin(); it != hypotheses_.end(); it++) {
        if (!it->valid)
          continue;

        double d = it->diff(o);

          // We use an inlier/outlier/even-worse-outlier model for the scoring function rho:
          if (d > inlierThreshold_) {
              // "Regular" outlier above the plane

          } else if (d < -inlierThreshold_) {
              // "Even worse" outlier below the plane, rho = - belowFactor
              //it->score -= belowFactor_;
          } else {
              // inlier
              it->score += 1;
          }
      }

      i++;

      // Evaluate preemption function to decide whether we can discard some hypotheses now.
      unsigned int prnext = preemption(i);
      if (prnext != pr) {
          // Time to discard a few hypotheses: Select best hypotheses
          std::nth_element(hypotheses_.begin(),hypotheses_.begin()+prnext,hypotheses_.end(),Plane::compare);
          // Now the first prnext elements of h contain the best, erase the rest
          hypotheses_.erase(hypotheses_.begin()+prnext,hypotheses_.end());
      }
      pr = prnext;
  } // preemptive scoring is done

  // Select the best hypothesis of possibly more than one remaining:
  std::nth_element(hypotheses_.begin(),hypotheses_.begin()+1,hypotheses_.end(),Plane::compare);

  Plane bestPlane = hypotheses_[0];
  double anglebest = std::acos((double) lastPlane_.n.dot(hypotheses_[0].n));
  if (angularTolerance_ > 0.0 &&
       anglebest > angularTolerance_)
  {
      bestPlane.valid = false;
      std::cout << "angle diff of bestPlane too large: " << anglebest << std::endl;
  }
  std::cout << "bestPlane.n: " << bestPlane.n << std::endl;

  return bestPlane;
}

Plane PlaneFinder::refinePlane(const std::vector<Eigen::Vector3d>& points, const std::vector<bool>& inlier)
{
    std::vector<Eigen::Vector3d> p;
    extractInliers(points, inlier, p);

//    cout << __PRETTY_FUNCTION__ << "using points: " << p.size() << endl;

    bestFit_ = lastPlane_;

    std::vector<double> weights(points.size(), 1.0);

    const int nIters = 5;
    for (int iter = 0; iter < nIters; iter++) {
        Eigen::Vector3d mean;
        Eigen::Matrix3d cov;
        computeMeanAndCovariance(p, weights, mean, cov);

        // Extract the smallest eigenvalue and its eigenvector
        Eigen::Vector3d::Scalar eigenValue;
        Eigen::Vector3d eigenVector;
        pcl::eigen33(cov, eigenValue, eigenVector);

        bestFit_.setPlaneFromPoint(eigenVector, mean);

        computeWeights(p, mean, cov, weights); // better without when finding a ground at each kf, why?
    }

    // enforce to be close to last ground plane
    if (lastPlane_.valid){
      double hightdiff = lastPlane_.n.dot(bestFit_.mean_ - lastPlane_.mean_);
      if (fabs(hightdiff) > hightTolerance_)
        bestFit_.valid = false;

      std::cout << "hightdiff: " << hightdiff << std::endl;
    }

    return bestFit_;
}

void PlaneFinder::extractInliers(const std::vector<Eigen::Vector3d>& points, const std::vector<bool>& inlier,
                    std::vector<Eigen::Vector3d>& pointsOut)
{
    pointsOut.clear();

    for (uint i = 0; i < points.size(); i++) {
        if (inlier[i])
            pointsOut.push_back(points[i]);
    }
}

void PlaneFinder::computeWeights(const std::vector<Eigen::Vector3d>& points,
                                          const Eigen::Vector3d& mean,
                                          const Eigen::Matrix3d& cov,
                                          std::vector<double>& weights)
{
    // Compute weights
    for (unsigned int m = 0; m < points.size(); m++) {
        Eigen::Vector3d diffMean = points[m] - mean;
        double maha = std::sqrt(diffMean.transpose()*cov.inverse()*diffMean);

        weights[m] = (bestFit_.isBelow(points[m]) ? belowFactor_ : 1)*weight(maha);
    }
}

void PlaneFinder::computeMeanAndCovariance(const std::vector<Eigen::Vector3d>& points,
                                                    const std::vector<double>& weights,
                                                    Eigen::Vector3d& mean,
                                                    Eigen::Matrix3d& cov)
{
    // Compute robust mean
    mean.setZero();
    cov.setZero();
    double sumW = 0.0;
    double sumW2 = 0.0;
    for (unsigned int m = 0; m < points.size(); m++) {
        mean += points[m]*weights[m];
        sumW += weights[m];
    }
    mean /= sumW;

    // Compute robust covariance matrix
    for (unsigned int m = 0; m < points.size(); m++) {
        Eigen::Vector3d diff = points[m] - mean;
        double w2 = weights[m]*weights[m];
        cov += w2*diff*diff.transpose();
        sumW2 += w2;
    }
    cov /= (sumW2 - 1.0);
}

