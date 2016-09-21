#include <vector>
#include <math.h>

/*
 This code was developed by Alan Soares
 Github - https://github.com/alanssoares
*/

typedef struct Point3D {
  double x;
  double y;
  double z;
} Point3D;

bool pointsEqual(Point3D p1, Point3D p2);
double length(Point3D point);
Point3D subtract(Point3D a, Point3D b);
double getDistancePointToPoint(Point3D p1, Point3D p2);
double getDistancePointToSegment(Point3D p, Point3D p1, Point3D p2);
std::vector<Point3D> simplify(std::vector<Point3D> points, double tolerance, bool highestQuality);
std::vector<Point3D> simplifyRadialDist(std::vector<Point3D> points, double sqTolerance);
std::vector<Point3D> simplifyDouglasPeucker(std::vector<Point3D> points, double sqTolerance);
std::vector<Point3D> simplifyDPStep(std::vector<Point3D> points, int first, int last, double sqTolerance, std::vector<Point3D> simplified);

bool pointsEqual(Point3D p1, Point3D p2){
    return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z;
}

double length(Point3D point){
    return sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
}

Point3D subtract(Point3D a, Point3D b){
    a.x = b.x - a.x;
    a.y = b.y - a.y;
    a.z = b.z - a.z;
    return a;
}

double getDistancePointToPoint(Point3D p1, Point3D p2){
    return length(subtract(p1, p2));
}

double getDistancePointToSegment(Point3D p, Point3D p1, Point3D p2){
    double x = p1.x,
        y = p1.y,
        z = p1.z,
        dx = p2.x - x,
        dy = p2.y - y,
        dz = p2.z - z;

    if (dx != 0 || dy != 0 || dz != 0) {
        double t = ((p.x - x) * dx + (p.y - y) * dy + (p.z - z) * dz) / (pow(dx,2) + pow(dy,2) + pow(dz,2));
        if (t > 1.0) {
            x = p2.x;
            y = p2.y;
            z = p2.z;
        } else if (t > 0.0) {
            x += dx * t;
            y += dy * t;
            z += dz * t;
        }
    }

    dx = p.x - x;
    dy = p.y - y;
    dz = p.z - z;

    return pow(dx,2) + pow(dy,2) + pow(dz,2);
}

std::vector<Point3D> simplify(std::vector<Point3D> points, double tolerance, bool highestQuality){
    double sqTolerance = pow(tolerance, 2);

    if(points.size() <= 2) {
        return points;
    }

    if(highestQuality){
        points = simplifyRadialDist(points, sqTolerance);
    }

    return simplifyDouglasPeucker(points, sqTolerance);
}

std::vector<Point3D> simplifyRadialDist(std::vector<Point3D> points, double sqTolerance){
    Point3D prevPoint, point;
    std::vector<Point3D> newPoints;
    double sqDistance = 0.0;
    size_t n = points.size();
    prevPoint = points.front();
    newPoints.push_back(prevPoint);

    for (int i = 1; i < n; i++) {
        point = points[i];
        sqDistance = getDistancePointToPoint(point, prevPoint);
        if (sqDistance > sqTolerance) {
            newPoints.push_back(point);
            prevPoint = point;
        }
    }

    if (!pointsEqual(prevPoint, point)) {
        newPoints.push_back(point);
    }

    return newPoints;
}

std::vector<Point3D> simplifyDouglasPeucker(std::vector<Point3D> points, double sqTolerance){
    std::vector<Point3D> simplified;
    simplified.push_back(points.front());
    simplified = simplifyDPStep(points, 1, points.size(), sqTolerance, simplified);
    simplified.push_back(points.back());
    return simplified;
}

std::vector<Point3D> simplifyDPStep(std::vector<Point3D> points, int first, int last, double sqTolerance, std::vector<Point3D> simplified){
    double maxSqDist = sqTolerance;
    int index = -1;

    for (int i = first + 1; i < last; i++) {
        double sqDist = getDistancePointToSegment(points[i], points[first], points[last]);
        if (sqDist > maxSqDist) {
            index = i;
            maxSqDist = sqDist;
        }
    }

    if (maxSqDist > sqTolerance) {
        if (index - first > 1) {
            simplified = simplifyDPStep(points, first, index, sqTolerance, simplified);
        }

        simplified.push_back(points[index]);

        if (last - index > 1) {
            simplified = simplifyDPStep(points, index, last, sqTolerance, simplified);
        }
    }

    return simplified;
}

int main(int argc, char const *argv[]) {
  std::vector<Point3D> points;

  /*
   Create here your points and add to the vector points
   Ex:

   Point3D p1;
   Point3D p2;

   points.push_back(p1);
   points.push_back(p2);

  */

  //Test with low quality
  std::vector<Point3D> resultA = simplify(points, 0.001, false);
  cout<<"Before: "<<points.size()<<" After with low: "<<resultA.size()<<endl;

  //Test with high quality
  std::vector<Point3D> resultB = simplify(points, 0.001, true);
  cout<<"Before: "<<points.size()<<" After with high: "<<resultB.size()<<endl;

  return 0;
}
