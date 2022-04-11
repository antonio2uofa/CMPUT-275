// Name: Antonio Martin-ozimek
// SID: 1658820
// CCID: antonio2
// Assignment #1: Part 2

#include <iostream>
#include <cassert>
#include <fstream>
#include <string>
#include <list>

#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "wdigraph.h"
#include "dijkstra.h"
#define _MSG_MAX_LENGTH 22

struct Point {
    long long lat, lon;
};

// returns the manhattan distance between two points
long long manhattan(const Point& pt1, const Point& pt2) {
  long long dLat = pt1.lat - pt2.lat, dLon = pt1.lon - pt2.lon;
  return abs(dLat) + abs(dLon);
}

// finds the id of the point that is closest to the given point "pt"
int findClosest(const Point& pt, const unordered_map<int, Point>& points) {
  pair<int, Point> best = *points.begin();

  for (const auto& check : points) {
    if (manhattan(pt, check.second) < manhattan(pt, best.second)) {
      best = check;
    }
  }
  return best.first;
}

// read the graph from the file that has the same format as the "Edmonton graph" file
void readGraph(const string& filename, WDigraph& g, unordered_map<int, Point>& points) {
  ifstream fin(filename);
  string line;

  while (getline(fin, line)) {
    // split the string around the commas, there will be 4 substrings either way
    string p[4];
    int at = 0;
    for (auto c : line) {
      if (c == ',') {
        // start new string
        ++at;
      }
      else {
        // append character to the string we are building
        p[at] += c;
      }
    }

    if (at != 3) {
      // empty line
      break;
    }

    if (p[0] == "V") {
      // new Point
      int id = stoi(p[1]);
      assert(id == stoll(p[1]));  // sanity check: asserts if some id is not 32-bit
      points[id].lat = static_cast<long long>(stod(p[2])*100000);
      points[id].lon = static_cast<long long>(stod(p[3])*100000);
      g.addVertex(id);
    }
    else {
      // new directed edge
      int u = stoi(p[1]), v = stoi(p[2]);
      g.addEdge(u, v, manhattan(points[u], points[v]));
    }
  }
}

int create_and_open_fifo(const char * pname, int mode) {
  // creating a fifo special file in the current working directory
  // with read-write permissions for communication with the plotter
  // both proecsses must open the fifo before they can perform
  // read and write operations on it
  if (mkfifo(pname, 0666) == -1) {
    cout << "Unable to make a fifo. Ensure that this pipe does not exist already!" << endl;
    exit(-1);
  }

  // opening the fifo for read-only or write-only access
  // a file descriptor that refers to the open file description is
  // returned
  int fd = open(pname, mode);

  if (fd == -1) {
    cout << "Error: failed on opening named pipe." << endl;
    exit(-1);
  }

  return fd;
}

/*
the main function builds the graph, calls the dijkstra function,
creates the inpipe, the outpipe, and writes the coordinates between
start and end to outpipe
*/
int main() {
  WDigraph graph;
  unordered_map<int, Point> points;

  const char *inpipe = "inpipe";
  const char *outpipe = "outpipe";

  // Open the two pipes
  int in = create_and_open_fifo(inpipe, O_RDONLY);
  cout << "inpipe opened..." << endl;
  int out = create_and_open_fifo(outpipe, O_WRONLY);
  cout << "outpipe opened..." << endl;

  // build the graph
  readGraph("server/edmonton-roads-2.0.1.txt", graph, points);

  // initialize the variables
  Point sPoint, ePoint;
  string msg = "";
  char current;
  int count = 0;
  vector<long long> pipe_pts;
  bool done = false;
  char delimiter = '\n';
  char finalchar = 'E';

  // we read the latitude and longitude from inpipe
  while (!done) {
    while (true) {
      // read one character in at a time
      read(in, &current, 1);

      // break when the newline character is read
      if (current == '\n') {
        pipe_pts.push_back(static_cast<long long>(stod(msg)*100000));
        msg = "";
        count++;
        break;

      // close pipes when Q is sent
      } else if (current == 'Q') {
        done = true;
        close(in);
        unlink(inpipe);
        close(out);
        unlink(outpipe);
        break;
      }

      // store the first input of each line in vector
      if (current == ' ') {
        pipe_pts.push_back(static_cast<long long>(stod(msg)*100000));
        msg = "";
      } else {
        // add character to end of string
        msg.push_back(current);
      }
    }

    // first if ensures we get the start point then returns to inner while loop
    if (count < 2) {
      // initialize start point
      sPoint.lat = pipe_pts[0];
      sPoint.lon = pipe_pts[1];

    // else means we have collected both points
    } else {
      // reset count, initialize end point
      count = 0;
      ePoint.lat = pipe_pts[2];
      ePoint.lon = pipe_pts[3];
      // clear vector
      pipe_pts.clear();
      int start = findClosest(sPoint, points);
      int end = findClosest(ePoint, points);

      // run dijkstra's algorithm, this is the unoptimized version that
      // does not stop when the end is reached but it is still fast enough
      unordered_map<int, PIL> tree;
      dijkstra(graph, start, tree);

      // no path so write 'E' to outpipe
      if (tree.find(end) == tree.end()) {
        write(out, &finalchar, 1);
        write(out, &delimiter, 1);

      // create path
      } else {
        list<int> path;
        while (end != start) {
          path.push_front(end);
          end = tree[end].first;
        }
        path.push_front(start);

        // index through path
        for (int v : path) {
          // turn long long into double
          double outlat = static_cast<double>(points[v].lat);
          double outlon = static_cast<double>(points[v].lon);

          // return to 100000th, remove last digit
          string latstr = to_string(outlat/100000);
          string lonstr = to_string(outlon/100000);
          latstr.pop_back();
          lonstr.pop_back();

          // write concatenated string to outpipe
          string output = latstr + " " + lonstr;
          write(out, output.c_str(), output.size());
          write(out, &delimiter, 1);
        }

        // write 'E' and '\n' to outpipe
        write(out, &finalchar, 1);
        write(out, &delimiter, 1);
      }
    }
  }
  return 0;
}
