//---------------------------------------------------------------------------

#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <string.h>

#include <terragear/clipper.hpp>
#include <terragear/tg_polygon.hxx>
#include <terragear/tg_shapefile.hxx>

//---------------------------------------------------------------------------

using namespace std;
using namespace ClipperLib;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

inline long64 Round(double val)
{
  if ((val < 0)) return (long64)(val - 0.5); else return (long64)(val + 0.5);
}
//------------------------------------------------------------------------------

int PrecisionFirstValue(const char * filename)
{
  char line[80];
  FILE *f = fopen(filename, "r");
  if (!f) return 0;

  if (fgets(line, 80, f) == 0) {
    fclose(f);
    return 0; //skip poly count
  }

  if (fgets(line, 80, f) == 0) {
    fclose(f);
    return 0; //skip length first polygon
  }

  if (fgets(line, 80, f) == 0) {
    fclose(f);
    return 0; //get coords first vertex
  }
  
  fclose(f);

  int i = 0;
  while (line[i] >= ' ' && line[i] != '.') i++;
  if (line[i] != '.') return 0;
  i++;
  int j = i;
  while (line[j] >= '0' && line[j] <= '9') j++;
  return j - i;
}
//------------------------------------------------------------------------------

bool IsBlankLine(char* line)
{
  while (*line) 
    if (*line > ' ') return false;
    else line++;
  return true;
}
//------------------------------------------------------------------------------

bool LoadFromFile(Polygons &ppg, char * filename, double scale)
{
  ppg.clear();

  FILE *f = fopen(filename, "r");
  if (!f) return false;
  int polyCnt, vertCnt;
  double X, Y;

  if (fscanf(f, "%d", &polyCnt) == 1 && polyCnt > 0)
  {
    ppg.resize(polyCnt);
    for (int i = 0; i < polyCnt; i++) {
      if (fscanf(f, "%d", &vertCnt) != 1 || vertCnt <= 0) break;
      ppg[i].resize(vertCnt);
      for (int j = 0; j < vertCnt; j++) {
        if (fscanf(f, "%lf%*[, ]%lf", &X, &Y) != 2) break;
        ppg[i][j].X = Round(X * scale);
        ppg[i][j].Y = Round(Y * scale);
        
        char junk [80];
        fgets(junk, 80, f);
      }
    }
  }
  fclose(f);
  return true;
}
//------------------------------------------------------------------------------

bool LoadFromFile2(Polygons &ppg, char * filename, double scale)
{
  ppg.clear();
  FILE *f = fopen(filename, "r");
  if (!f) return false;
  char line [80];
  double X, Y;
  Polygon pg;
  while (fgets(line, sizeof line, f))
  {
    if (IsBlankLine(line)) 
    {
      if (pg.size() > 0) 
      {
        ppg.push_back(pg);
        pg.clear();
      }
      continue;
    }
    if (sscanf(line, "%lf%*[, ]%lf", &X, &Y) != 2) break;
    pg.push_back(IntPoint(Round(X * scale), Round(Y * scale)));
  }
  if (pg.size() > 0) ppg.push_back(pg);
  fclose(f);
  return true;
}
//------------------------------------------------------------------------------

void SaveToFile(char *filename, Polygons &pp, int precision)
{
  double scale = std::pow(double(10), precision);
  FILE *f = fopen(filename, "w");
  if (!f) return;
  fprintf(f, "%d\n", pp.size());
  for (unsigned i = 0; i < pp.size(); ++i)
  {
    fprintf(f, "%d\n", pp[i].size());
    if (precision != 0) {
      for (unsigned j = 0; j < pp[i].size(); ++j)
        fprintf(f, "%.*lf, %.*lf,\n",
          precision, (double)pp[i][j].X /scale,
          precision, (double)pp[i][j].Y /scale);
    }
    else
    {
      for (unsigned j = 0; j < pp[i].size(); ++j)
        fprintf(f, "%lld, %lld,\n", pp[i][j].X, pp[i][j].Y );
    }
  }
  fclose(f);
}
//---------------------------------------------------------------------------

void SaveToFile(char *filename, ExPolygons &pp, int precision)
{
  double scale = std::pow(double(10), precision);
  FILE *f = fopen(filename, "w");
  if (!f) return;
  int cnt = 0;
  for (unsigned i = 0; i < pp.size(); ++i)
    cnt += pp[i].holes.size() +1;
  fseek(f, 0, SEEK_SET);
  fprintf(f, "%d\n", cnt);
  for (unsigned i = 0; i < pp.size(); ++i)
  {
    fprintf(f, "%d\n", pp[i].outer.size());
    if (precision != 0) {
      for (unsigned j = 0; j < pp[i].outer.size(); ++j)
        fprintf(f, "%.*lf, %.*lf,\n",
          precision, (double)pp[i].outer[j].X /scale,
          precision, (double)pp[i].outer[j].Y /scale);
    }
    else
    {
      for (unsigned j = 0; j < pp[i].outer.size(); ++j)
        fprintf(f, "%lld, %lld,\n", pp[i].outer[j].X, pp[i].outer[j].Y );
    }
    for (unsigned k = 0; k < pp[i].holes.size(); ++k)
    {
      fprintf(f, "%d\n", pp[i].holes[k].size());
      if (precision != 0) {
        for (unsigned j = 0; j < pp[i].holes[k].size(); ++j)
          fprintf(f, "%.*lf, %.*lf,\n",
            precision, (double)pp[i].holes[k][j].X /scale,
            precision, (double)pp[i].holes[k][j].Y /scale);
      }
      else
      {
        for (unsigned j = 0; j < pp[i].holes[k].size(); ++j)
          fprintf(f, "%lld, %lld,\n", pp[i].holes[k][j].X, pp[i].holes[k][j].Y );
      }
    }
  }
  fclose(f);
}

static void SaveToFileOstream(char *filename, Polygons &pp)
{
    std::ofstream file;

    file.open (filename);
    file << pp;
    file.close();
}

//---------------------------------------------------------------------------

#ifdef __BORLANDC__
int _tmain(int argc, _TCHAR* argv[])
#else
int main(int argc, char* argv[])
#endif
{
  if (argc < 4)
  {
    cout << "\nUSAGE:\n"
      << "clipper subject_file clip_file CLIPTYPE [SUBJ_FILL CLIP_FILL] [SVG]\n\n"
      << "  CLIPTYPE  = i[ntersection] or u[nion] or d[ifference] or x[or], and\n"
      << "  SUBJ_FILL & CLIP_FILL = evenodd or nonzero (default = nonzero)\n"
      << "  SVG = Create SVG file\n"
      << "  where parameters in [] are optional\n\n";

    //format 1
    //cout << "\nFORMAT OF INPUT AND OUTPUT FILES:\n"
    //  << "Polygon Count {must be on the first line}\n"
    //  << "Vertex Count {of first polygon}\n"
    //  << "X[,] Y[,] {first vertex}\n"
    //  << "X[,] Y[,] {next vertex}\n"
    //  << "...\n"
    //  << "Vertex Count {of second polygon, if there is one}\n"
    //  << "X[,] Y[,] {first vertex of second polygon}\n"
    //  << "...\n\n";

    //format 2
    cout << "\nFORMAT OF INPUT AND OUTPUT FILES:\n"
      << "Each polygon is separated by a blank line \n"
      << "X[,] Y[,] {first vertex}\n"
      << "X[,] Y[,] {next vertex}\n"
      << "...\n"
      << "\n"
      << "X[,] Y[,] {first vertex of second polygon}\n"
      << "...\n\n";

    cout << "\nEXAMPLE:\n"
      << "clipper subj.txt clip.txt u evenodd evenodd SVG\n\n";
  return 1;
  }

  int precision = PrecisionFirstValue(argv[1]);
  double scale = std::pow(double(10), precision);

  bool show_svg =
    ((argc > 4 && strcasecmp(argv[4], "SVG") == 0) ||
    (argc > 5 && strcasecmp(argv[5], "SVG") == 0) ||
    (argc > 6 && strcasecmp(argv[6], "SVG") == 0));

  Polygons subject, clip;

  if (!LoadFromFile2(subject, argv[1], scale))
  {
    cerr << "\nCan't open the file " << argv[1]
      << " or the file format is invalid.\n";
    return 1;
  }
  if (!LoadFromFile2(clip, argv[2], scale))
  {
    cerr << "\nCan't open the file " << argv[2]
      << " or the file format is invalid.\n";
    return 1;
  }

  if (show_svg) {
    tgShapefile::FromClipper( subject, "./clptst_subject", "subject", "subject" );
    tgShapefile::FromClipper( clip, "./clptst_clip", "clip", "clip" );
  }
  
  ClipType clipType;
  switch (toupper(argv[3][0])) {
    case 'X': clipType = ClipType::Xor; break;
    case 'U': clipType = ClipType::Union; break;
    case 'D': clipType = ClipType::Difference; break;
    default: clipType = ClipType::Intersection;
  }

  PolyFillType subj_pft = PolyFillType::NonZero, clip_pft = PolyFillType::NonZero;
  if (argc > 4&& strcasecmp(argv[4], "EVENODD") == 0)
      subj_pft = PolyFillType::EvenOdd;
  if (argc > 5 && strcasecmp(argv[5], "EVENODD") == 0)
      clip_pft = PolyFillType::EvenOdd;

  cout << "\nclipping ... \n";

  Clipper c;
  c.AddPolygons(subject, PolyType::Subject);
  c.AddPolygons(clip, PolyType::Clip);
  Polygons solution;

//  double elapsed = 0;
//  _LARGE_INTEGER qpf, qpc1, qpc2;
//  bool HPMEnabled = QueryPerformanceFrequency(&qpf);
//  if (HPMEnabled) QueryPerformanceCounter(&qpc1);

  bool succeeded = c.Execute(clipType, solution, subj_pft, clip_pft);

//  if (HPMEnabled) {
//    QueryPerformanceCounter(&qpc2);
//    elapsed = double(qpc2.QuadPart - qpc1.QuadPart) / qpf.QuadPart;
//    cout << "\nEllapsed: " << elapsed;
//  }

  for ( int i=0; i<solution.size(); i++ ) {
      Polygon contour = solution[i];
      if ( Orientation( contour ) ) {
          cout << "solution contour " << i << " is true \n";
      } else {
          cout << "solution contour " << i << " is false \n";
      }
  }

  if (succeeded) {
      SaveToFile("solution.txt", solution, precision);
      SaveToFileOstream("solution_ostream", solution );
      cout << "succeeded.\nSolution saved to - solution.txt.\n\n";
      cout << "\n";

      if( show_svg ) {
        cout << "Generating shapefile\n";
        tgShapefile::FromClipper( solution, "./clptst_solution", "solution", "solution" );
      }
  } else
      cout << "failed.\n\n";
  return 0;
}
//---------------------------------------------------------------------------
