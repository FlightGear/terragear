You need the Gnu Triangulated Surfaces library (GTS) installed on your
system to build portions of TerraGear.  GTS provides the needed tools
and infrastructure to impliment our terrain simplification approach.
We modeled our approach after the approach outlined in Michael
Garland's paper here:

    http://graphics.cs.uiuc.edu/~garland/software/terra.html

You can get the latest version of GTS from:

    http://gts.sourceforge.net/

More information:

  GTS stands for the GNU Triangulated Surface Library. It is an Open
  Source Free Software Library intended to provide a set of useful
  functions to deal with 3D surfaces meshed with interconnected
  triangles. The source code is available free of charge under the
  Free Software LGPL license.

  The code is written entirely in C with an object-oriented approach
  based mostly on the design of GTK+. Careful attention is paid to
  performance related issues as the initial goal of GTS is to provide
  a simple and efficient library to scientists dealing with 3D
  computational surface meshes.

  A brief summary of its main features:

    * Simple object-oriented structure giving easy access to
      topological properties. 
    * 2D dynamic Delaunay and constrained Delaunay triangulations.
    * Robust geometric predicates (orientation, in circle) using fast
      adaptive floating point arithmetic (adapted from the fine work
      of Jonathan R. Shewchuk).
    * Robust set operations on surfaces (union, intersection, difference).
    * Surface refinement and coarsening (multiresolution models).
    * Dynamic view-independent continuous level-of-detail.
    * Preliminary support for view-dependent level-of-detail.
    * Bounding-boxes trees and Kd-trees for efficient point location
      and collision/intersection detection.
    * Graph operations: traversal, graph partitioning.
    * Metric operations (area, volume, curvature ...).
    * Triangle strips generation for fast rendering. 
